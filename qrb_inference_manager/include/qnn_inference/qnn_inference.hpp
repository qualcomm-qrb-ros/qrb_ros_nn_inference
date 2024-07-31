#ifndef QRB_INFERENCE_MANAGER_QNN_INFERENCE_HPP_
#define QRB_INFERENCE_MANAGER_QNN_INFERENCE_HPP_

#include "qrb_inference.hpp"
#include "qnn_inference/qnn_inference_impl.hpp"

namespace qrb::inference_mgr
{

class QnnInference : public QrbInference
{
public:
  QnnInference(const std::string &backend_option, const std::string &model_path);
  ~QnnInference() = default;
  StatusCode inference_init() override;
  StatusCode inference_graph_init() override;
  StatusCode inference_execute(const std::vector<uint8_t> &input_tensor_data) override;
  const std::vector<OutputTensor> get_output_tensors() override;

private:
  std::unique_ptr<QnnInferenceImpl> qnn_inference_impl_;
}; // class QnnInference

class QnnInferenceFromFile : public QrbInference
{
public:
  QnnInferenceFromFile(const std::string &backend_option, const std::string &model_path);
  ~QnnInferenceFromFile() = default;
  StatusCode inference_init() override;
  StatusCode inference_graph_init() override;
  StatusCode inference_execute(const std::vector<uint8_t> &input_tensor_data) override;
  const std::vector<OutputTensor> get_output_tensors() override;

private:
  std::unique_ptr<QnnInferenceFromFileImpl> qnn_inference_fromfile_impl_;
}; // class QnnInferenceFromFile

} // namespace qrb::inference_mgr

#endif