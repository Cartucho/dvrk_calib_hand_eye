# dvrk_calib_arms_to_camera

-- REPO UNDER CONSTRUCTION --

The goal of this code is to estimate the geometric transformations between a camera and the robotic arms of a da Vinci dVRK robot. E.g., the transformation from the left-stereo camera to the PSM1's base. This is a fundamental calibration for tasks such as visual servoeing, where you want to move the PSM given input endoscopic images.

Surgical instrument and projected skeleton after calibrating the transformation:
TODO: giff with tool moving

Assumptions:
- I will assume that you will be using a `8.3 [mm]` surgical instrument, which is the standard diameter of the shaft;
- You will have `"neon green"` sticker paper, which you can buy on Amazon or any another store. This sticker paper will be used to wrap a marker around the shaft of the surgical instrument;
- You have a monocular camera (or stereo) at a fixed position. If you are using the ECM, you can lock the ECM's joints to a fixed state;

## Step 1 - Camera calibration

First, we will calculate both the `distortion` and `intrinsic camera parameters` of the camera that you will be using as pivot. In the case of a stereo endoscope, we suggest that you use the left-stereo camera as pivot. It is **very important that the camera is well calibrated**, since this will have a significant impact in the accuracy of the estimated transformations. If you have already accurately calibrated your camera, please skip to step 2, otherwise follow the following steps:

TODO: create marker with 1.25 of aspect ratio
// min distance = 30 * focal_length_mm
// focal_length_mm = (sensor_width_mm / sensor_width_pixels) * focal_length_pixels

Print the PDF file of the camera calibration pattern. Please check that the printed pattern width is indeed X [mm] and the height is Y [mm], to verify that your printer is not automatically scaling the pattern. Attach the pattern to a rigid surface (never bend a calibration pattern!), and take pictures of the calibration pattern in different poses, while respecting the following rules:
- Focus the cameras to the working distance (in MIS the working distance is usually between 10 [cm] and 20 [cm] TODO: find the right working distance);
  [TODO: add picture]
- At this working distance, take 50 pictures in total, with the pattern at different orientations;
- Adjust the brightness of the light source, so that the corners are fully visible;
  [TODO: add picture]
- Keep the camera in a static position;
- Do not move the pattern while taking a picture! This would cause motion blur. Also, do not hold the pattern in your hand, your hand is not that steady. Just make sure that it is static while taking each picture;
  [TODO: add picture]
- Capture all the pattern' corners by keeping the pattern inside the field of view of the camera;
  [TODO: add picture]
- The pattern should cover as much of the field of view as possible!
  [TODO: add picture]
- Do not take pictures with the pattern parallel to the image plane, add small inclinations. Keep the inclination less than 45 degrees;
  [TODO: add picture]

TODO: sample calibration images

To take pictures run the following code
`python main.py --task camera_take_pictures`

`python main.py --task camera_calib`

TODO: ?show sample calibration result?

## Step 2 - Record data + move the robotic arm

First, you will need to print the green marker on sticker paper, to wrap the marker around the shaft of the surgical instrument. 

TODO:

## Step 3 - Get green marker pose in recorded data

TODO:

## Step 4 - Calculate the transformation

TODO:

## FAQ

- `I moved to set-up joints, what should I do?`
If you move the set-up joints you need to repeat the calibration.

 - `I moved the camera, what should I do?`
 You need to put the camera back in the original pose, otherwise repeat the calibration. If you are using the ECM, all you have to do is put the ECM back to the same joint state as you initially had, when you did the transformation calibration! An alternative solution, is to do the advanced calibration decribed below. With the advanced calibration you can update the transformations even when you move the ECM, as long as you do not move the set-up joints.


## Advanced calibration: How to update the transformation in real-time if I move the ECM?

If you move the ECM, you change the ECMS' joints state, and therefore the transformation between the camera and the base of the PSM's also changes.
TODO: explain that there is a rigid transformation