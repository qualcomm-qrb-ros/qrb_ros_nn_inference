// Copyright (c) 2026 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_post_process_with_dma/qrb_ros_post_process_with_dma_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <dlfcn.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>

#include "rclcpp_components/register_node_macro.hpp"

namespace qrb_ros_post_process_with_dma
{

// Helper function to create directory if it doesn't exist
static bool ensure_directory_exists(const std::string & path)
{
  struct stat info;
  if (stat(path.c_str(), &info) != 0) {
    // Directory doesn't exist, try to create it
    if (mkdir(path.c_str(), 0755) != 0) {
      return false;
    }
  } else if (!(info.st_mode & S_IFDIR)) {
    // Path exists but is not a directory
    return false;
  }
  return true;
}

// Helper function to extract directory path from full file path
static std::string get_directory_path(const std::string & filepath)
{
  size_t pos = filepath.find_last_of("/\\");
  if (pos != std::string::npos) {
    return filepath.substr(0, pos);
  }
  return "";
}

QrbRosPostProcessWithDmaNode::QrbRosPostProcessWithDmaNode(const rclcpp::NodeOptions & options)
: Node("qrb_ros_post_process_with_dma", options)
{
  in_w_ = this->declare_parameter<int>("input_width", 518);
  in_h_ = this->declare_parameter<int>("input_height", 518);

  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth_map", 10);

  infer_sub_ = this->create_subscription<TensorList>(
    "qrb_inference_output_tensor", 10,
    std::bind(&QrbRosPostProcessWithDmaNode::infer_callback, this, std::placeholders::_1));

  // Initialize RPCMEM for memory management
  using RpcMemInitFn_t = void (*)(void);

  rpcmem_lib_ = ::dlopen("libcdsprpc.so", RTLD_NOW | RTLD_LOCAL);
  if (nullptr == rpcmem_lib_) {
    throw std::runtime_error("dlopen(libcdsprpc.so) failed");
  }

  auto rpcmem_init = (RpcMemInitFn_t)::dlsym(rpcmem_lib_, "rpcmem_init");
  if (nullptr == rpcmem_init) {
    ::dlclose(rpcmem_lib_);
    rpcmem_lib_ = nullptr;
    throw std::runtime_error("resolve rpcmem_init failed");
  }

  rpcmem_init();
  RCLCPP_INFO(this->get_logger(), "rpcmem_init done for post-process");
  RCLCPP_INFO(this->get_logger(), "qrb_ros_post_process_with_dma node started");
}

QrbRosPostProcessWithDmaNode::~QrbRosPostProcessWithDmaNode()
{
  if (rpcmem_lib_ != nullptr) {
    ::dlclose(rpcmem_lib_);
    rpcmem_lib_ = nullptr;
  }
  RCLCPP_INFO(this->get_logger(), "qrb_ros_post_process_with_dma node destroyed, resources cleaned up");
}

cv::Mat QrbRosPostProcessWithDmaNode::postprocess(const float * depth, int orig_w, int orig_h, float scale,
    int pad_left, int pad_top) const
{
  // Input depth is expected to be [518,518,1] float32
  cv::Mat depth_518(in_h_, in_w_, CV_32FC1, const_cast<float *>(depth));

  // Undo pad+resize like python:
  // - resize by 1/scale
  cv::Mat rescaled;
  cv::resize(depth_518, rescaled, cv::Size(), 1.0 / scale, 1.0 / scale, cv::INTER_LINEAR);

  const int scaled_pad_left = static_cast<int>(std::round(static_cast<float>(pad_left) / scale));
  const int scaled_pad_top = static_cast<int>(std::round(static_cast<float>(pad_top) / scale));

  cv::Rect roi(scaled_pad_left, scaled_pad_top, orig_w, orig_h);
  roi &= cv::Rect(0, 0, rescaled.cols, rescaled.rows);

  cv::Mat cropped = rescaled(roi);

  // normalize to [0,255] and colorize
  cv::Mat normalized_u8;
  cv::normalize(cropped, normalized_u8, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  cv::Mat colored;
  cv::applyColorMap(normalized_u8, colored, cv::COLORMAP_INFERNO);
  return colored;
}

void QrbRosPostProcessWithDmaNode::infer_callback(const TensorList & msg)
{
  // Static counter for saving images
  static int frame_count = 0;
  frame_count++;

  RCLCPP_INFO(this->get_logger(), "========== POST-PROCESS Frame #%d START ==========", frame_count);
  RCLCPP_INFO(this->get_logger(), "infer_callback called. tensors=%zu", msg.tensor_list.size());

  // Expect output as DMA-BUF
  for (const auto & t : msg.tensor_list) {
    RCLCPP_INFO(this->get_logger(), "[MEMORY] Received tensor: fd=%d, size=%u, ptr=0x%lx",
                t.dmabuf_fd, t.dmabuf_size, t.dmabuf_ptr);

    if (t.dmabuf_fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Received non-DMA output (data copy path). size=%zu", t.data.size());
      continue;
    }

    // Map output fd using mmap
    void * mapped_addr = mmap(nullptr, t.dmabuf_size, PROT_READ | PROT_WRITE, MAP_SHARED, t.dmabuf_fd, 0);
    if (mapped_addr == MAP_FAILED) {
      RCLCPP_ERROR(this->get_logger(), "Failed to mmap output dmabuf fd=%d, size=%u", t.dmabuf_fd, t.dmabuf_size);
      continue;
    }
    RCLCPP_INFO(this->get_logger(), "[MEMORY] mmap successful: mapped_addr=%p", mapped_addr);

    const auto * depth = reinterpret_cast<const float *>(
      reinterpret_cast<const uint8_t *>(mapped_addr) + t.dmabuf_offset);

    // Save raw depth data (518x518 float32) for verification
    cv::Mat depth_518(in_h_, in_w_, CV_32FC1, const_cast<float *>(depth));

    // Normalize to [0,255] for visualization
    cv::Mat depth_normalized;
    cv::normalize(depth_518, depth_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    // Save raw depth map (grayscale)
    std::string depth_gray_filename = "/home/ubuntu/ros-ws/nn_output/" + std::to_string(frame_count) + "_gray.png";
    std::string output_dir = get_directory_path(depth_gray_filename);
    if (!output_dir.empty() && !ensure_directory_exists(output_dir)) {
      RCLCPP_WARN(this->get_logger(), "Failed to create output directory: %s", output_dir.c_str());
    }
    if (cv::imwrite(depth_gray_filename, depth_normalized)) {
      RCLCPP_INFO(this->get_logger(), "Saved raw depth map to: %s", depth_gray_filename.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save raw depth map to: %s", depth_gray_filename.c_str());
    }

    // Postprocess and create colored depth map
    cv::Mat colored = postprocess(depth, last_orig_w_, last_orig_h_, last_scale_, last_pad_left_, last_pad_top_);

    // Save colored depth map
    std::string depth_color_filename = "/home/ubuntu/ros-ws/nn_output/" + std::to_string(frame_count) + "_color.png";
    // Directory should already exist from grayscale save, but check anyway
    std::string output_dir_color = get_directory_path(depth_color_filename);
    if (!output_dir_color.empty() && !ensure_directory_exists(output_dir_color)) {
      RCLCPP_WARN(this->get_logger(), "Failed to create output directory: %s", output_dir_color.c_str());
    }
    if (cv::imwrite(depth_color_filename, colored)) {
      RCLCPP_INFO(this->get_logger(), "Saved colored depth map to: %s", depth_color_filename.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to save colored depth map to: %s", depth_color_filename.c_str());
    }

    sensor_msgs::msg::Image out_msg;
    out_msg.header = msg.header;
    out_msg.height = static_cast<uint32_t>(colored.rows);
    out_msg.width = static_cast<uint32_t>(colored.cols);
    out_msg.encoding = "bgr8";
    out_msg.is_bigendian = false;
    out_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(colored.step);
    out_msg.data.assign(colored.data, colored.data + colored.total() * colored.elemSize());

    depth_pub_->publish(out_msg);
    RCLCPP_INFO(this->get_logger(), "Published depth_map (from DMA-BUF output fd=%d)", t.dmabuf_fd);
    RCLCPP_INFO(this->get_logger(), "=== Frame %d processing complete ===", frame_count);

    // Unmap the memory
    if (munmap(mapped_addr, t.dmabuf_size) != 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to munmap output dmabuf");
    }

    // Free the RPCMEM buffer using rpcmem_free with the original pointer
    if (t.dmabuf_ptr != 0) {
      using RpcMemFreeFn_t = void (*)(void *);
      auto rpcmem_free = (RpcMemFreeFn_t)::dlsym(rpcmem_lib_, "rpcmem_free");

      if (rpcmem_free != nullptr) {
        void * rpc_ptr = reinterpret_cast<void *>(t.dmabuf_ptr);
        rpcmem_free(rpc_ptr);
        RCLCPP_INFO(this->get_logger(), "Freed RPCMEM buffer for fd=%d, ptr=%p", t.dmabuf_fd, rpc_ptr);
      } else {
        RCLCPP_WARN(this->get_logger(), "rpcmem_free symbol not found");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "dmabuf_ptr is 0, cannot free memory");
    }
  }
}

}  // namespace qrb_ros_post_process_with_dma

RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros_post_process_with_dma::QrbRosPostProcessWithDmaNode)
