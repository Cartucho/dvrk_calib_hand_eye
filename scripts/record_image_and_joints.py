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


class RecordImageAndJoints:
  def __init__(self, out_joint, out_dir_im, rostopic_cam, rostopic_psm):
    self.out_joint = out_joint
    self.out_dir_im = out_dir_im
    self.clean_dir()
    self.im = None

    self.counter = 0
    self.joints = {}

    self.data_sub1 = message_filters.Subscriber(rostopic_cam, Image)
    self.data_sub2 = message_filters.Subscriber(rostopic_psm, JointState)

    self.bridge = CvBridge()
    self.ats = message_filters.ApproximateTimeSynchronizer([self.data_sub1,
                                                            self.data_sub2],
                                                            queue_size=400,
                                                            slop=0.05)
    self.ats.registerCallback(self.sync_callback)


  def clean_dir(self):
    # Delete data from previous recording
    if os.path.isfile(self.out_joint):
        os.remove(self.out_joint)
    if os.path.isdir(self.out_dir_im):
        shutil.rmtree(self.out_dir_im)
    os.mkdir(self.out_dir_im)


  def sync_callback(self, msg_im, msg_joint_state):
    self.joint = [float(np_float) for np_float in msg_joint_state.position]
    try:
        self.im = self.bridge.imgmsg_to_cv2(msg_im, "bgr8")
    except CvBridgeError as e:
        print(e)


def main(args):
  rospy.init_node('joints_and_image_recorder', anonymous=True)
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
  rostopic_psm = data_yaml['rostopic']['sub_psm']
  # Create `data/` folder to store data
  if not os.path.isdir(path_data):
    os.mkdir(path_data)
  out_joint = os.path.join(path_data, "joint_recorded.yaml")
  out_dir_im = os.path.join(path_data, "cam_recorded") 
  rec = RecordImageAndJoints(out_joint, out_dir_im, rostopic_cam, rostopic_psm)
  window_name = "Save pose"
  cv2.namedWindow(window_name, cv2.WINDOW_KEEPRATIO)
  while not rospy.is_shutdown():
    if rec.im is not None:
      cv2.imshow(window_name, rec.im)
      pressed_key = cv2.waitKey(3)
      ind_str = "{:04}".format(rec.counter)
      """ Save if user press [ENTER] & Exit if user press [q] """
      if pressed_key == 13: # 13 is the Enter key
        print('recorded pose {}!'.format(ind_str))
        tmp_dict = {}
        tmp_dict['PSM'] = {'joints': rec.joint}
        rec.joints[ind_str] = tmp_dict
        with open(rec.out_joint, 'w') as yaml_file:
          yaml.dump(rec.joints, yaml_file, default_flow_style=False)
        im_path = os.path.join(rec.out_dir_im, "{}.png".format(ind_str))
        cv2.imwrite(im_path, rec.im)
        rec.counter += 1
      elif pressed_key == 113: # User pressed 'q' to quit
        rospy.signal_shutdown("Done")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
