#!/usr/bin/env python3

from tabnanny import check
from matplotlib.colors import NoNorm
import rospy
from sensor_msgs.msg import Image as Img
from cv_bridge import CvBridge, CvBridgeError
from parcel_msgs.srv import parcel_poses,parcel_posesResponse
from parcel_msgs.msg import parcel_pose

import os
import sys
import copy
import numpy as np
from PIL import Image, ImageDraw
from pathlib import Path

import torch
import torch.backends.cudnn as cudnn

from models.common import DetectMultiBackend
from utils.torch_utils import select_device
from utils.general import (LOGGER, check_img_size, non_max_suppression, cv2, scale_coords)
from utils.augmentations import letterbox
from utils.plots import Annotator, colors

class parcel_converter:
  def __init__(self):
    self.weights = '/home/jo/catkin_ws/src/yolov5_ros/yolov5/face.pt'
    self.device = select_device('0')
    self.model = DetectMultiBackend(self.weights, device=self.device, dnn=False, data=None, fp16=False)
    self.stride, self.names, self.pt = self.model.stride, self.model.names, self.model.pt
    self.imgsz = (640, 640)
    self.imgsz = check_img_size(self.imgsz, s=self.stride)
    self.conf_thres = 0.7

    self.bridge = CvBridge()
    self.parcel_pose = parcel_pose()
    self.parcel_server = rospy.Service("parcel_find_server", parcel_poses, self.callback)
    rospy.loginfo("SERVER IS RUNNIG!!")
    rospy.loginfo("WAITING FOR SERVICE......\n")

  def callback(self, data):
    try:
      img = self.bridge.imgmsg_to_cv2(data.image, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
    rospy.loginfo("SERVIEC ACCPED!!")
    parcel_pose_list = parcel_posesResponse()

    im = letterbox(img, self.imgsz, stride=self.stride, auto=self.pt)[0]
    im = im.transpose((2, 0, 1))[::-1]
    im = np.ascontiguousarray(im)
    im = torch.from_numpy(im).to(self.device)
    im = im.half() if self.model.fp16 else im.float()
    im /= 255
    if len(im.shape) == 3:
      im = im[None]

    #Inference
    pred = self.model(im, augment=False, visualize=False)

    #NMS
    pred  = non_max_suppression(pred, self.conf_thres, 0.45, None, False, max_det=300)

    #Process predictions
    for i, det in enumerate(pred):
      img_P = Image.fromarray(img.astype(np.uint8)) if isinstance(img, np.ndarray) else img  # from np
      if len(det):
        # Rescale boxes from img_size to img size
        det[:, :4] = scale_coords(im.shape[2:], det[:, :4], img.shape).round()

        # Write results
        for *xyxy, conf, cls in reversed(det):
          c = int(cls)
          ImageDraw.Draw(img_P).rectangle(xyxy, width=3, outline ="red")  # plot
          temp = []
          for x in xyxy:
            temp.append(x.tolist())
          temp_to = copy.deepcopy(self.xyxy2xywh(temp, self.names[c]))
          parcel_pose_list.parcel_pose.append(temp_to)

    # Stream results
    img_P.show()
    
    rospy.loginfo(f'{parcel_pose_list}')
    return parcel_posesResponse(parcel_pose_list.parcel_pose)

  def xyxy2xywh(self,x,cls):
    # Convert nx4 boxes from [x1, y1, x2, y2] to [x, y, w, h] where xy1=top-left, xy2=bottom-right
    self.parcel_pose.cls = cls
    self.parcel_pose.x = int((x[0] + x[2]) / 2)  # x center
    self.parcel_pose.y = int((x[1] + x[3]) / 2)  # y center
    self.parcel_pose.w = int(x[2] - x[0])  # width
    self.parcel_pose.h = int(x[3] - x[1])  # height
    return self.parcel_pose

def main(args):
  pc = parcel_converter()
  rospy.init_node('parcel_find_server', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main(sys.argv)
