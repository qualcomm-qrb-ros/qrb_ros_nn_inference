#ifndef QRB_INFERENCE_HPP_
#define QRB_INFERENCE_HPP_

#include <memory>
#include <string>
#include <iostream>
#include <vector>

namespace qrb::inference_mgr
{

template<typename... Args>
void QRB_DEBUG(Args&&... args) {
  (std::cout << ... << args) << std::endl;
}

enum class StatusCode {
  SUCCESS,
  FAILURE,
  FAILURE_INPUT_LIST_EXHAUSTED,
  FAILURE_SYSTEM_ERROR,
  FAILURE_SYSTEM_COMMUNICATION_ERROR,
  QNN_FEATURE_UNSUPPORTED
};

struct OutputTensor {
  std::vector<uint8_t> output_tensor_data_;
  std::string output_tensor_name_;
  std::vector<uint32_t> output_tensor_shape_;
};

class QrbInference
{
public:
  QrbInference() = default;
  virtual ~QrbInference() = default;
  virtual StatusCode inference_init() = 0;
  virtual StatusCode inference_graph_init() = 0;
  virtual StatusCode inference_execute(const std::vector<uint8_t> &input_tensor_data) = 0;
  virtual const std::vector<OutputTensor> get_output_tensors() = 0;
}; // class QrbInference

} // namespace qrb::inference_mgr

#endif
