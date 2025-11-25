# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import os
import numpy as np
from PIL import Image

image_path = os.environ['HOME'] + "/ros-ws/src/qrb_ros_nn_inference/test/qrb_ros_pre_process/image/image.jpg"

with Image.open(image_path) as img:
    img_resized = img.resize((640, 640))
    img_array = np.array(img_resized, dtype=np.float32)
    output_filename = os.path.splitext(image_path)[0] + '.raw'
    img_array.tofile(output_filename)

print("Images have been resized and saved in raw format: {}".format(output_filename))