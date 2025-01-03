#include <iostream>
#include <vector>

#include "qrb_inference_manager.hpp"

int main()
{
  // std::string model_path{"model.tflite"};
  // std::string backend_option{"gpu"};

  // std::string model_path{"model.bin"};
  // std::string backend_option{"libQnnHtp.so"};

  std::string model_path{ "model.so" };
  std::string backend_option{ "libQnnCpu.so" };

  auto qrb_inference_mgr =
      std::make_unique<qrb::inference_mgr::QrbInferenceManager>(model_path, backend_option);

  std::vector<uint8_t> input_data;  // input tensor data for model inference
  if (false == qrb_inference_mgr->inference_execute(input_data)) {
    std::cout << "ERROR: Inference failed!" << std::endl;
  }

  std::vector<qrb::inference_mgr::OutputTensor> output_data;  // output tensor, result of inference
  output_data = qrb_inference_mgr->get_output_tensors();

  return 0;
}
