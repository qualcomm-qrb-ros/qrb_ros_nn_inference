#include "qrb_ros_pre_process_with_dma/qrb_ros_pre_process_with_dma_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <dlfcn.h>

#include <opencv2/imgcodecs.hpp>

#include "rclcpp_components/register_node_macro.hpp"

namespace qrb_ros_pre_process_with_dma
{

static inline size_t element_count_nhwc(int n, int h, int w, int c)
{
  return static_cast<size_t>(n) * static_cast<size_t>(h) * static_cast<size_t>(w) *
         static_cast<size_t>(c);
}

QrbRosPreProcessWithDmaNode::QrbRosPreProcessWithDmaNode(const rclcpp::NodeOptions & options)
: Node("qrb_ros_pre_process_with_dma", options)
{
  in_w_ = this->declare_parameter<int>("input_width", 518);
  in_h_ = this->declare_parameter<int>("input_height", 518);

  // Subscribe images (same behavior as sample_depth_estimation):
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&QrbRosPreProcessWithDmaNode::image_callback, this, std::placeholders::_1));

  infer_pub_ = this->create_publisher<TensorList>("qrb_inference_input_tensor", 10);

  input_bytes_ = element_count_nhwc(1, in_h_, in_w_, in_c_) * sizeof(float);

  // RPCMEM APIs (ION) on Qualcomm Linux
  using RpcMemAllocFn_t = void * (*)(int, uint32_t, int);
  using RpcMemFreeFn_t = void (*)(void *);
  using RpcMemToFdFn_t = int (*)(void *);
  using RpcMemInitFn_t = void (*)(void);

  rpcmem_lib_ = ::dlopen("libcdsprpc.so", RTLD_NOW | RTLD_LOCAL);
  if (nullptr == rpcmem_lib_) {
    throw std::runtime_error("dlopen(libcdsprpc.so) failed");
  }

  auto rpcmem_init = (RpcMemInitFn_t)::dlsym(rpcmem_lib_, "rpcmem_init");
  auto rpcmem_alloc = (RpcMemAllocFn_t)::dlsym(rpcmem_lib_, "rpcmem_alloc");
  auto rpcmem_free = (RpcMemFreeFn_t)::dlsym(rpcmem_lib_, "rpcmem_free");
  auto rpcmem_to_fd = (RpcMemToFdFn_t)::dlsym(rpcmem_lib_, "rpcmem_to_fd");
  if (nullptr == rpcmem_init || nullptr == rpcmem_alloc || nullptr == rpcmem_free ||
      nullptr == rpcmem_to_fd) {
    ::dlclose(rpcmem_lib_);
    rpcmem_lib_ = nullptr;
    throw std::runtime_error("resolve rpcmem symbols failed");
  }

  // Required on some Linux targets so that subsequent rpcmem_alloc buffers can be mapped via FastRPC.
  rpcmem_init();
  RCLCPP_INFO(this->get_logger(), "rpcmem_init done");

  constexpr int RPCMEM_HEAP_ID_SYSTEM = 25;
  constexpr uint32_t RPCMEM_DEFAULT_FLAGS = 1;

  /**
  * Defination: void* rpcmem_alloc(int heapid, uint32 flags, int size);
  * Allocate a buffer via ION and register it with the FastRPC framework.
  * @param[in] heapid  Heap ID to use for memory allocation.
  * @param[in] flags   ION flags to use for memory allocation.
  * @param[in] size    Buffer size to allocate.
  * @return            Pointer to the buffer on success; NULL on failure.
  */
  // ! Allocate a buffer via ION and register it with the FastRPC framework
  input_ptr_ = rpcmem_alloc(RPCMEM_HEAP_ID_SYSTEM, RPCMEM_DEFAULT_FLAGS, static_cast<int>(input_bytes_));
  if (nullptr == input_ptr_) {
    ::dlclose(rpcmem_lib_);
    rpcmem_lib_ = nullptr;
    throw std::runtime_error("rpcmem_alloc failed");
  }
  input_fd_ = rpcmem_to_fd(input_ptr_);
  if (input_fd_ < 0) {
    rpcmem_free(input_ptr_);
    input_ptr_ = nullptr;
    ::dlclose(rpcmem_lib_);
    rpcmem_lib_ = nullptr;
    throw std::runtime_error("rpcmem_to_fd failed");
  }

  RCLCPP_INFO(this->get_logger(), "Allocated RPCMEM input: fd=%d bytes=%zu", input_fd_, input_bytes_);
  RCLCPP_INFO(this->get_logger(), "qrb_ros_pre_process_with_dma node started");
}

QrbRosPreProcessWithDmaNode::~QrbRosPreProcessWithDmaNode()
{
  if (rpcmem_lib_ != nullptr) {
    using RpcMemFreeFn_t = void (*)(void *);
    auto rpcmem_free = (RpcMemFreeFn_t)::dlsym(rpcmem_lib_, "rpcmem_free");
    if (rpcmem_free != nullptr && input_ptr_ != nullptr) {
      rpcmem_free(input_ptr_);
      input_ptr_ = nullptr;
      input_fd_ = -1;
    }
    ::dlclose(rpcmem_lib_);
    rpcmem_lib_ = nullptr;
  }
  RCLCPP_INFO(this->get_logger(), "qrb_ros_pre_process_with_dma node destroyed, resources cleaned up");
}

cv::Mat QrbRosPreProcessWithDmaNode::nv12_to_bgr(const uint8_t * nv12, int width, int height) const
{
  cv::Mat yuv(height * 3 / 2, width, CV_8UC1, const_cast<uint8_t *>(nv12));
  cv::Mat bgr;
  cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
  return bgr;
}

cv::Mat QrbRosPreProcessWithDmaNode::preprocess(const cv::Mat & bgr, std::vector<float> & nhwc,
    int & orig_w, int & orig_h, float & scale, int & pad_left, int & pad_top) const
{
  orig_h = bgr.rows;
  orig_w = bgr.cols;

  const int dst_h = in_h_;
  const int dst_w = in_w_;

  const float h_ratio = static_cast<float>(dst_h) / static_cast<float>(orig_h);
  const float w_ratio = static_cast<float>(dst_w) / static_cast<float>(orig_w);
  scale = std::min(h_ratio, w_ratio);

  const int new_h = static_cast<int>(std::floor(orig_h * scale));
  const int new_w = static_cast<int>(std::floor(orig_w * scale));

  const int pad_h = dst_h - new_h;
  const int pad_w = dst_w - new_w;

  const int pad_top_local = pad_h / 2;
  const int pad_left_local = pad_w / 2;
  const int pad_bottom = pad_h - pad_top_local;
  const int pad_right = pad_w - pad_left_local;

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_CUBIC);

  cv::Mat padded;
  cv::copyMakeBorder(
    resized, padded, pad_top_local, pad_bottom, pad_left_local, pad_right, cv::BORDER_CONSTANT,
    cv::Scalar(0, 0, 0));

  // Convert to float32 [0,1] and NHWC
  cv::Mat f32;
  padded.convertTo(f32, CV_32FC3, 1.0 / 255.0);

  nhwc.resize(element_count_nhwc(1, dst_h, dst_w, in_c_));
  std::memcpy(nhwc.data(), f32.data, nhwc.size() * sizeof(float));

  pad_left = pad_left_local;
  pad_top = pad_top_local;

  return f32;
}

void QrbRosPreProcessWithDmaNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv::Mat bgr;
  if (msg->encoding == "bgr8") {
    bgr = cv_bridge::toCvCopy(*msg, "bgr8")->image;
  } else if (msg->encoding == "nv12") {
    bgr = nv12_to_bgr(msg->data.data(), static_cast<int>(msg->width), static_cast<int>(msg->height));
  } else {
    RCLCPP_ERROR(this->get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
    return;
  }

  std::vector<float> nhwc;
  int ow = 0, oh = 0, pad_left = 0, pad_top = 0;
  float scale = 1.0f;
  (void)preprocess(bgr, nhwc, ow, oh, scale, pad_left, pad_top);

  // ! Write image data into RPCMEM buffer
  std::memcpy(input_ptr_, nhwc.data(), input_bytes_);

  TensorList tl;
  Tensor t;
  t.data_type = 2;  // float32
  t.name = "depth_anything_input_tensor";
  t.shape = {1u, static_cast<uint32_t>(in_h_), static_cast<uint32_t>(in_w_), static_cast<uint32_t>(in_c_)};

  // Toggle between DMA-BUF and data copy modes for performance testing
  // Set USE_DMABUF to true for zero-copy mode (requires same process)
  // Set USE_DMABUF to false for data copy mode (works cross-process)
  constexpr bool USE_DMABUF = true;  // Temporarily disabled for debugging

  if (USE_DMABUF) {
    // DMA-BUF mode (for testing - will fail across processes)
    t.dmabuf_fd = input_fd_;
    t.dmabuf_size = static_cast<uint32_t>(input_bytes_);
    t.dmabuf_offset = 0;
    t.dmabuf_ptr = reinterpret_cast<uint64_t>(input_ptr_);  // Pass pointer address for memory management
    // t.data left empty for zero-copy
  } else {
    // Data copy mode (working across processes)
    t.dmabuf_fd = -1;
    t.dmabuf_size = 0;
    t.dmabuf_offset = 0;
    // Copy data to ROS message for cross-process transfer
    t.data.resize(input_bytes_);
    std::memcpy(t.data.data(), input_ptr_, input_bytes_);
  }

  // Print timestamp before publishing (millisecond precision)
  auto now = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  RCLCPP_INFO(this->get_logger(), "[TIMESTAMP] Before publish: %ld ms", ms);

  // debug: show topic wiring (namespace + absolute topic)
  RCLCPP_INFO(this->get_logger(), "Publishing to topic: %s", infer_pub_->get_topic_name());
  tl.tensor_list.push_back(std::move(t));
  infer_pub_->publish(tl);

  RCLCPP_INFO(this->get_logger(), "Published input tensor via data copy: size=%zu bytes", input_bytes_);
}

}  // namespace qrb_ros_pre_process_with_dma

RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros_pre_process_with_dma::QrbRosPreProcessWithDmaNode)
