#!/usr/bin/env python3

import argparse
import time
from pathlib import Path
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random

import ros_numpy
from weedbot_msgs.msg import Object, Detection

sys.path.insert(0, '../yolov5')
from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.torch_utils import select_device, time_synchronized

def callback(data):
    global model, names, device, half, pub

    t1 = time_synchronized()

    img = ros_numpy.image.image_to_numpy(data)
    img = img.transpose((2, 0, 1))
    img = torch.from_numpy(img).to(device)

    det_msg = Detection()
    det_msg.frame_id = data.header.frame_id
    det_msg.img_seq = data.header.seq

    img = img.half() if half else img.float()  # uint8 to fp16/32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0
    if img.ndimension() == 3:
        img = img.unsqueeze(0)

    # Inference
    pred = model(img, augment=opt.augment)[0]

    # Apply NMS
    pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
    t2 = time_synchronized()
    s = ""

    rospy.loginfo(str(s) + " Done. " + str(t2 - t1) + "s")
    det_msg.inf_time = t2 - t1

    # Process detections
    for i, det in enumerate(pred):
        s += '%gx%g ' % img.shape[2:]  # print string
        if len(det):
            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
                rospy.loginfo("!!!" + str(s))
            # Write results
            for *xyxy, conf, cls in reversed(det):
                obj_msg = Object()
                xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4))).view(-1).tolist()
                rospy.logdebug("### <" + str(conf) + "> " + str(xyxy) + " - " + str(xywh))
                obj_msg.x1 = xyxy[0]
                obj_msg.y1 = xyxy[1]
                obj_msg.x2 = xyxy[2]
                obj_msg.y2 = xyxy[3]
                obj_msg.width = xywh[2]
                obj_msg.height = xywh[3]
                obj_msg.confidence = conf
                obj_msg.object_cls = cls
                det_msg.objects.append(obj_msg)

    pub.publish(det_msg)

def detect():
    global model, names, device, half, pub
    weights, imgsz = opt.weights, opt.img_size

    rospy.init_node('detector', anonymous=True)

    rospy.loginfo("Starting")
    # Initialize
    device = select_device(opt.device)
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    model = attempt_load(weights, map_location=device)  # load FP32 modelimgsz
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size
    if half:
        model.half()  # to FP16

    names = model.module.names if hasattr(model, 'module') else model.names

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once

    rospy.loginfo("Done")
    rospy.Subscriber("/weeds/detection/image_resized", Image, callback)
    pub = rospy.Publisher("/weeds/detection/raw", Detection, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default='weights/best_1000.pt', help='model.pt path(s)')
    parser.add_argument('--img-size', type=int, default=416, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.4, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='0', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
    parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')
    parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
    opt = parser.parse_args()

    with torch.no_grad():
        detect()
