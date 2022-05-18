# dvrk_calib_hand_eye

-- REPO UNDER CONSTRUCTION --

The goal of this code is to estimate the geometric transformations between a camera and the robotic arms of a da Vinci dVRK robot. E.g., the transformation from the left-stereo camera to the PSM1's base. This is a fundamental calibration for tasks such as visual servoeing, where you want to move the PSM given input endoscopic images.

Surgical instrument and projected skeleton after calibrating the transformation:
TODO: giff with tool moving

Assumptions:
- I will assume that you will be using a `8.35 [mm]` surgical instrument, which is the standard diameter of the shaft;
- You will have `"neon green"` sticker paper, which you can buy on Amazon or any another store. This sticker paper will be used to wrap a marker around the shaft of the surgical instrument;

  <img src="https://user-images.githubusercontent.com/15831541/165297939-ebcb03ef-c781-4ad2-bd5b-16712d0d018d.png" width="20%">

- You have a monocular (or stereo) `camera at a fixed position`. If you are using the ECM, you can lock the ECM's joints to a fixed state;

## Step 0

Clone this repo:

`git clone git@github.com:Cartucho/dvrk_calib_hand_eye.git`

Clone the `cylmarker` sub-module:

`TODO`

Build it:

`catkin build dvrk_calib_hand_eye`

## Step 1 - Camera calibration

First, we will calculate both the `distortion` and `intrinsic camera parameters` of the camera that you will be using as pivot. In the case of a stereo endoscope, we suggest that you use the left-stereo camera as pivot. It is **very important that the camera is well calibrated**, since this will have a significant impact in the accuracy of the estimated transformations. If you have already accurately calibrated your camera just edit the values in the file [config.yaml](https://github.com/Cartucho/dvrk_calib_arms_to_camera/blob/main/config.yaml) skip to step 2, otherwise follow the [camera_calibration.md](https://github.com/Cartucho/dvrk_calib_arms_to_camera/blob/main/camera_calibration.md) instructions.

## Step 2 - Record data (by moving the surgical instrument)

First, you will need to print the green marker on sticker paper, to wrap the marker around the shaft of the surgical instrument.
Launch both the camera and the PSM.

After doing that, let's record the green marker at multiple poses:

`rosrun dvrk_calib_hand_eye record_image_and_joints.py`

## Step 3 - Get green marker pose in recorded data

`Home` the PSM.

`rosrun dvrk_calib_hand_eye record_image_and_joints.py`

## Step 4 - Calculate the transformation

TODO:

## FAQ

- `I moved to set-up joints, what should I do?`
If you move the set-up joints you need to repeat the calibration.

 - `I moved the camera, what should I do?`
 You need to put the camera back in the original pose, otherwise repeat the calibration. If you are using the ECM, all you have to do is put the ECM back to the same joint state as you initially had, when you did the transformation calibration! An alternative solution, is to do the advanced calibration decribed below. With the advanced calibration you can update the transformations even when you move the ECM, as long as you do not move the set-up joints.

- `What if I want to use rectified images instead of the ones captured directly from the camera?`
When you rectify the camera rotates, therefore you need to update the transformation according to that rotation. Another alternative is to rectify the images and repeat step 2 to 4.

## Advanced calibration: How to update the transformation in real-time if I move the ECM?

If you move the ECM, you change the ECMS' joints state, and therefore the transformation between the camera and the base of the PSM's also changes.
TODO: explain that there is a rigid transformation
