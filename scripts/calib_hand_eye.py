#!/usr/bin/env python
import sys
import os
import glob
import math
import yaml

import rospy
import rospkg
import cv2 as cv
import numpy as np


def calibrate_ax_yb(AA, BB):
    """
     Solves the problem AX=YB using the formulation of

     `Simultaneous Robot/World and Tool/Flange Calibration
     by Solving Homogeneous Transformation Equations of the form AX=YB` M. Shah

     Based on: http://math.loyola.edu/~mili/Calibration/
    """
    n = len(AA)
    A = np.zeros((9*n,18))
    T = np.zeros((9,9))
    b = np.zeros((9*n,1))
    for i in range(0,n):
        Ra = AA[i][0:3,0:3]
        Rb = BB[i][0:3,0:3]
        T += np.kron(Rb,Ra)

    (u,s,v)=np.linalg.svd(T)
    v = v.T # in python v.T = v in matlab
    x = v[:,0]
    y = u[:,0]

    X = np.reshape(x,(3,3)).T
    X = np.sign(np.linalg.det(X))/abs(np.linalg.det(X))**(1/3)*X
    (u,s,v)=np.linalg.svd(X)
    v = v.T
    X = np.matmul(u,v.T)

    Y = np.reshape(y,(3,3)).T
    Y = np.sign(np.linalg.det(Y))/abs(np.linalg.det(Y))**(1/3)*Y
    (u,s,v)=np.linalg.svd(Y)
    v = v.T
    Y = np.matmul(u,v.T)

    A = np.zeros((3*n, 6))
    b = np.zeros((3*n, 1))

    for i in range(0,n):
        A[3*i:3*i+3,:] = np.concatenate((-AA[i][0:3,0:3], np.eye(3)),axis = 1)
        b[3*i:3*i+3,:] = AA[i][0:3,3].reshape(3,1)- np.matmul(np.kron(BB[i][0:3,3].T, np.eye(3)), Y.T.reshape((9,1)))

    t = np.matmul(np.linalg.pinv(A),b)

    X = np.concatenate((np.concatenate((X,t[0:3]),axis = 1),np.array([[0,0,0,1]])),axis = 0)
    Y = np.concatenate((np.concatenate((Y,t[3:6]),axis = 1),np.array([[0,0,0,1]])),axis = 0)
    return X, Y


def load_np_txt(file_path):
    return np.loadtxt(file_path, delimiter=',')


def transf_DH_modified(alpha, a, theta, d):
    trnsf = np.array([[math.cos(theta), -math.sin(theta), 0., a],
                      [math.sin(theta) * math.cos(alpha), math.cos(theta) * math.cos(alpha), -math.sin(alpha), -d * math.sin(alpha)],
                      [math.sin(theta) * math.sin(alpha), math.cos(theta) * math.sin(alpha), math.cos(alpha), d * math.cos(alpha)],
                      [0., 0., 0., 1.]])
    return trnsf


def get_bPSM_T_j3(joint_value):
    LRcc = 0.4318
    """                               alpha   ,  a , theta                      , D                   """
    base_T_j1 = transf_DH_modified( np.pi*0.5 , 0. , joint_value[0] + np.pi*0.5 ,                  0. )
    j1_T_j2   = transf_DH_modified(-np.pi*0.5 , 0. , joint_value[1] - np.pi*0.5 ,                  0. )
    j2_T_j3   = transf_DH_modified( np.pi*0.5 , 0. ,                        0.  , joint_value[2]-LRcc )
    #bPSM_T_j3 = base_T_j1 @ j1_T_j2 @ j2_T_j3 #(requires a newer numpy version)
    bPSM_T_j3 = np.matmul(base_T_j1, np.matmul(j1_T_j2, j2_T_j3))
    return bPSM_T_j3


def calib_hand_eye(path_joints, path_images):
  A_raw = []
  with open(path_joints, "r") as f:
    A_raw = yaml.safe_load(f)

  A = []
  B = []
  # Convert pose messages into transformations
  for im_id, joints in A_raw.items():
    # Get Ai
    joints = joints["PSM"]["joints"]
    A_i = get_bPSM_T_j3(joints)
    # Get Bi
    B_i_path = os.path.join(path_images, "{}.txt".format(im_id))
    if os.path.isfile(B_i_path):
      B_i = load_np_txt(B_i_path)
      # Append
      A.append(A_i)
      B.append(B_i)
  if len(A) > 0:
    _, bPSM_T_c = calibrate_ax_yb(A, B)
    c_T_bPSM = np.linalg.inv(bPSM_T_c)
    print("cam_T_basePSM=\n{}".format(c_T_bPSM))
  else:
    print("No poses found, please make sure you did step 3!")


def main(args):
  rospy.init_node('calib_hand_eye', anonymous=True)
  rospack = rospkg.RosPack()
  path_package = rospack.get_path('dvrk_calib_hand_eye')
  path_data = os.path.join(path_package, 'data')
  path_images = os.path.join(path_data, 'cam_interpolated')
  path_joints = os.path.join(path_data, 'joint_interpolated.yaml')
  if os.path.isfile(path_joints):
    calib_hand_eye(path_joints, path_images)
  else:
    print("File not found: {}".format(path_joints))
    print("\t please run first: `rosrun dvrk_calib_hand_eye replay_interpolated.py`")


if __name__ == '__main__':
  main(sys.argv)
