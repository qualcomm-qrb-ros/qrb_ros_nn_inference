# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import numpy as np
import cv2
import os

def pre_process(input_file):
    orig = cv2.imread(input_file)
    resized = cv2.resize(orig, (640, 640), interpolation = cv2.INTER_AREA)
    return (orig, resized)

def post_process(yolo_data):
    def non_max_suppression(boxes, scores, thres):
        '''
            Transfer Pascal-VOC to COCO
            box[0]          : up-left x
            box[1]          : up-left y
            box[2] - box[0] : width
            box[3] - box[0] : height
        '''
        rects = [[box[0], box[1], box[2] - box[0], box[3] - box[1]] for box in boxes]
        nms_boxes = cv2.dnn.NMSBoxes(rects, scores, score_threshold=thres, nms_threshold=0.4)
        return nms_boxes

    thres = 0.5
    boxes = yolo_data[0]
    confidences = yolo_data[1]
    class_probs = yolo_data[2]

    obj_boxes   = []
    obj_scores  = []
    obj_classes = []

    for i in range(boxes.shape[1]):
        confidence = confidences[0, i]
        if confidence < thres :
            continue

        box = boxes[0, i]
        class_prob = class_probs[0, i]

        obj_boxes.append([int(box[0]), int(box[1]), int(box[2]), int(box[3])])
        obj_scores.append(confidence)
        obj_classes.append(class_prob)

    # NMS filter duplicate data
    box_indexs = non_max_suppression(obj_boxes, obj_scores, thres)

    output = {
        "boxes" :  [obj_boxes[i] for i in box_indexs],
        "scores":  [obj_scores[i] for i in box_indexs],
        "classes": [obj_classes[i] for i in box_indexs]
    }

    return output

def recover_size(boxes, orig_size, size):
    h_ratio = orig_size[0] / size[0]
    w_ratio = orig_size[1] / size[1]
    # print(f"h_ratio:{h_ratio} w_ratio:{w_ratio}")
    for box in boxes:
        box[0] = int(box[0] * w_ratio)
        box[1] = int(box[1] * h_ratio)
        box[2] = int(box[2] * w_ratio)
        box[3] = int(box[3] * h_ratio)

def draw_boxes(image, yolo_data):
    for index, box in enumerate(yolo_data["boxes"]):
        xmin, ymin, xmax, ymax = box
        cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)

        # add confidence label
        score = yolo_data["scores"][index]
        score = round(score, 2)
        cv2.putText(image, str(score) , (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

def main():
    workspace = os.environ['QRB_ROS_WS'] + "/src/qrb_ros_nn_inference/test/qrb_ros_post_process"
    result_dir = workspace + "/inference_result/"
    if not os.path.exists(result_dir):
        print(f"{result_dir} not exit!")
        exit(1)

    input_file = os.environ['QRB_ROS_WS'] + "/src/qrb_ros_nn_inference/test/qrb_ros_pre_process/image/image.jpg"
    orig_img, resized_img = pre_process(input_file)

    boxes = np.fromfile(result_dir + '/boxes.raw', dtype=np.float32).reshape((1, 8400, 4)) # boxes
    scores = np.fromfile(result_dir + '/scores.raw', dtype=np.float32).reshape((1, 8400)) # confidences
    class_idx = np.fromfile(result_dir + '/class_idx.raw', dtype=np.float32).reshape((1, 8400)) # classes
    yolo_data = [boxes, scores, class_idx]

    final_data = post_process(yolo_data)
    recover_size(final_data["boxes"], (orig_img.shape[:2]), (resized_img.shape[:2]))

    # visualization and save result
    draw_boxes(orig_img, final_data)
    cv2.imwrite(result_dir + "detection_result.jpg", orig_img)

if __name__ == "__main__":
    main()