#!/usr/bin/env python
import sys
import os
import shutil
import yaml

import cv2
import numpy as np

import rospy
import rospkg
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError


class RecordImage:
  def __init__(self, out_dir_im, rostopic_cam):
    self.out_dir_im = out_dir_im
    self.clean_dir()
    self.im = None

    self.counter = 0

    self.bridge = CvBridge()
    self.data_sub = rospy.Subscriber(rostopic_cam, Image, self.callback)


  def clean_dir(self):
    # Delete data from previous recording
    if os.path.isdir(self.out_dir_im):
      shutil.rmtree(self.out_dir_im)
    os.mkdir(self.out_dir_im)


  def callback(self, msg_im):
    try:
      self.im = self.bridge.imgmsg_to_cv2(msg_im, "bgr8")
    except CvBridgeError as e:
      print(e)


def main(args):
  rospy.init_node('image_recorder', anonymous=True)
  rospack = rospkg.RosPack()
  path_package = rospack.get_path('dvrk_calib_hand_eye')
  path_config = os.path.join(path_package, 'config.yaml')
  # Load rostopics form `config.yaml` file
  with open(path_config, 'r') as stream:
    try:
        data_yaml = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)
  path_data = os.path.join(path_package, 'data')
  rostopic_cam = data_yaml['rostopic']['sub_cam']
  # Create `data/` folder to store data
  if not os.path.isdir(path_data):
    os.mkdir(path_data)
  out_dir_im = os.path.join(path_data, "cam_calib") 
  rec = RecordImage(out_dir_im, rostopic_cam)
  window_name = "Save pose"
  cv2.namedWindow(window_name, cv2.WINDOW_KEEPRATIO)
  """
    Initialize camera calib pattern
  """
  n_cols = 23 - 1
  n_rows = 18 - 1
  size = (n_cols, n_rows) # size of checkerboard
  while not rospy.is_shutdown():
    if rec.im is not None:
      im_copy = rec.im.copy()
      gray = cv2.cvtColor(rec.im, cv2.COLOR_BGR2GRAY)
      ret, corners = cv2.findChessboardCorners(gray, size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
      if ret:
        cv2.drawChessboardCorners(im_copy, size, corners, ret)
      cv2.imshow(window_name, im_copy)
      pressed_key = cv2.waitKey(3)
      ind_str = "{:04}".format(rec.counter)
      """ Save if user press [ENTER] & Exit if user press [q] """
      if pressed_key == 13: # 13 is the Enter key
        if ret:
          print('recorded image {}!'.format(ind_str))
          im_path = os.path.join(rec.out_dir_im, "{}.png".format(ind_str))
          cv2.imwrite(im_path, rec.im)
          rec.counter += 1
        else:
          print('corners not detected, skipping image saving...')
      elif pressed_key == 113: # User pressed 'q' to quit
        rospy.signal_shutdown("Done")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
