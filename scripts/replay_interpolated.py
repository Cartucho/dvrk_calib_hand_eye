#!/usr/bin/env python
import os
import sys
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


class ReplayInterpolated:

  def __init__(self, rostopic_cam_sub,
                     rostopic_psm_sub,
                     rostopic_psm_pub,
                     n_steps,
                     in_joint,
                     out_im,
                     out_joint):
    self.load_recorded_joint_states(in_joint, n_steps)
    self.out_im = out_im
    self.out_joint = out_joint
    self.clean_dirs()

    self.pub = rospy.Publisher(rostopic_psm_pub, JointState, queue_size=10)

    self.data_sub1 = message_filters.Subscriber(rostopic_cam_sub, Image)
    self.data_sub2 = message_filters.Subscriber(rostopic_psm_sub, JointState)

    self.bridge = CvBridge()
    self.ats = message_filters.ApproximateTimeSynchronizer([self.data_sub1,
                                                            self.data_sub2],
                                                            queue_size=400,
                                                            slop=0.05)
    self.ats.registerCallback(self.sync_callback)
    self.playback()


  def clean_dirs(self):
    # Delete data from previous playback
    if os.path.isfile(self.out_joint):
      os.remove(self.out_joint)
    if os.path.isdir(self.out_im):
      shutil.rmtree(self.out_im)
    os.mkdir(self.out_im)


  def playback(self):
    joints = {}
    for ind, goal_joint in enumerate(self.goals_joint):
        print("Pose {}, joints:{}".format(ind, goal_joint[:3]))
        ind_str = "{:04}".format(ind)
        # Move arm
        rospy.sleep(0.5)
        joint_msg = self.create_goal_joint_msg(goal_joint)
        self.pub.publish(joint_msg)
        rospy.sleep(3.0) # Wait to stabilize arm
        # Save images
        if self.im is not None:
          im_path = "{}.png".format(ind_str)
          out_path = os.path.join(self.out_im, im_path)
          cv2.imwrite(out_path, self.im)
        # Save joints
        tmp_dict = {}
        tmp_dict["PSM"] = {"joints": self.joint}
        joints[ind_str] = tmp_dict
        with open(self.out_joint, 'w') as yaml_file:
          yaml.dump(joints, yaml_file, default_flow_style=False)
    rospy.signal_shutdown("Done")


  def create_goal_joint_msg(self, joint_position):
    joint_msg = JointState()
    joint_msg.header = Header()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw']
    joint_msg.position = joint_position
    joint_msg.velocity = []
    joint_msg.effort = []
    return joint_msg


  def sync_callback(self, msg_im, msg_joint_state):
    self.joint = [float(np_float) for np_float in msg_joint_state.position]
    try:
      self.im = self.bridge.imgmsg_to_cv2(msg_im, "bgr8")
    except CvBridgeError as e:
      print(e)


  def load_recorded_joint_states(self, in_joint, n_steps):
    joints = {}
    with open(in_joint, 'rb') as yaml_file:
        joints = yaml.safe_load(yaml_file)

    self.goals_joint = []
    """ Instead of going only throught the recorded joints add extra n_steps 
         in between each two successive joint states.
    """
    joints_init = joints["0000"]["PSM"]["joints"]
    j4_init = joints_init[3]
    j5_init = joints_init[4]
    j6_init = joints_init[5]
    for i in range(len(joints) - 1):
      ind_start = "{:04}".format(i)
      ind_end = "{:04}".format(i + 1)
      joints_start = joints[ind_start]["PSM"]["joints"]
      joints_end = joints[ind_end]["PSM"]["joints"]
      j1 = np.linspace(joints_start[0], joints_end[0], n_steps)
      j2 = np.linspace(joints_start[1], joints_end[1], n_steps)
      j3 = np.linspace(joints_start[2], joints_end[2], n_steps)
      j4 = np.repeat(j4_init, n_steps)
      j5 = np.repeat(j5_init, n_steps)
      j6 = np.repeat(j6_init, n_steps)
      for i in range(0, len(j1)):
      	self.goals_joint.append([j1[i], j2[i], j3[i], j4[i], j5[i], j6[i]])


def main(args):
  rospy.init_node('joints_and_image_interpolator', anonymous=True)
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
  rostopic_cam_sub = data_yaml['rostopic']['sub_cam']
  rostopic_psm_sub = data_yaml['rostopic']['sub_psm']
  rostopic_psm_pub = data_yaml['rostopic']['pub_psm']
  n_steps = 10
  in_joint = os.path.join(path_data, "joint_recorded.yaml") 
  out_im = os.path.join(path_data, "cam_interpolated")
  out_joint = os.path.join(path_data, "joint_interpolated.yaml")
  jp = ReplayInterpolated(rostopic_cam_sub,
                          rostopic_psm_sub,
                          rostopic_psm_pub,
                          n_steps,
                          in_joint,
                          out_im,
                          out_joint)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
