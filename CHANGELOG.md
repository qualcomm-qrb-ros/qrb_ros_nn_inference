# Changelog for package qrb_ros_nn_inference

## 2.3.0 (2026-07-15)
- add multi-graph shared-context inference node (QrbRosSharedInferenceNode)
- Contributors👉 Sam Freund

## 2.2.0 (2026-05-18)
- add HTP performance infrastructure initialization for binary model inference
- support QNN_HTP_NO_PERF env var to opt out of performance init
- Contributors👉 Na Song

## 2.1.2 (2026-02-04)
- optimize the code style
- Contributors👉 Na Song

## 2.1.1 (2026/02/03)
- fix bug of wrong graph_name
- support multiple input tensors
- Contributors👉 Na Song

## 2.0.0 (2026-01-30)
- support zero-copy transport for bin-model and HTP backend
- Contributors👉 Na Song

## 1.2.0 (2025-12-05)
- adapt Ubuntu
- Contributors👉 Na Song

## 1.1.1-jazzy (2025-08-11)

- adapt QNN_TENSOR_VERSION_2
- Contributors👉 Na Song

## 1.1.0-jazzy (2025-07-31)

- support Ubuntu
- Contributors👉 Na Song

## 1.0.0-jazzy (2025-05-19)

- first release on QCLinux based on ROS2 Jazzy
- Contributors👉 Na Song

## 0.5.0 (2025-05-16)

- support more data type(uint8) for input tensor
- Contributors👉 Na Song

## 0.4.0 (2025-05-15)

- support more data type(uint8) for output tensor
- Contributors👉 Xiao Li

## 0.3.1 (2025-05-07)

- fix bug during copy_graph_info()
- Contributors👉 Na Song

## 0.3.0 (2025-04-28)

- support multiple Qnn Graph
- Contributors👉 Na Song

## 0.2.0 (2025-01-02)

- optimize the compiling process of tensorflow-lite
- qnn_delegate_inference use tensorflow-lit C API
- Contributors👉 Na Song

## 0.1.1 (2024-11-13)

- optimize the setup of io tenosrs
- optimize the parameters of QrbInferenceManager()
- Contributors👉 Na Song

## 0.1.0 (2024-08-26)

- clean up all dependency of QnnSampleApp
- clean up the function of InferenceFromFile
- add the support of bin-format model
- Contributors👉 Na Song

## 0.0.0 (2024-07-10)

- Initial version release for ROS2 Humble
- Contributors👉 Na Song