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
