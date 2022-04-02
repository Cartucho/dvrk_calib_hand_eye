# dvrk_calib_arms_to_camera
The goal of this code is to estimate the geometric transformations between a camera and the robotic arms of a da Vinci dVRK robot. E.g., the transformation from the left-stereo camera to the PSM1's base. This is a fundamental calibration for tasks such as visual servoeing, where you want to move the PSM given what you process in the endoscopic image.

- I will assume that you have dVRK already set-up and running, and that you will be using a `8.3 [mm]` surgical instrument, which is the standard diameter of the shaft;
- You will need `"neon green"` sticker paper, which you can find it on Amazon. This sticker paper will be used to stick a marker around the shaft of the surgical instrument;
- You will also need a monocular or stereo camera at a fixed position (or with the ECM joints fixed at a static joint state);

## Step 1 - Camera calibration

First, we will calculate both the distortion and intrinsic camera parameters of the camera that you will be using as pivot. In the case of a stereo endoscope, we suggest that you use the left-stereo camera as pivot. It is **very important that the camera is well calibrated**, since this will have a significant impact in the accuracy of the estimated transformations. If you have already accurately calibrated your camera, please skip to step 2, otherwise follow the following steps:


## Step 2 - Record data while moving the robotic arm

First, you will need to print the green marker on sticker paper, to wrap the marker around the shaft of the surgical instrument.


## Step 3 - Get transformation from camera to green marker

## Step 4 - Calculate the transformation

## Advanced: How to update the transformation in real-time if I move the ECM?

If you move the ECM, you change the ECMS' joints state, and therefore the transformation between the camera and the base of the PSM's also changes.