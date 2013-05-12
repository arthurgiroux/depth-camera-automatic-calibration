depth-camera-automatic-calibration
==================================

Automatic calibration tool for depth and rgb camera

To build the project you may have to change the LDFLAG variable in the Makefile,
to determine the value of this variable you can use the command pkg-config --libs --cflags opencv

The pipeline is cut into three differents program.


=================
The first program is the tracking one and is use to get the position of the marker from different cameras view.

The program take the video input as the first parameter and the output directory for the calibration files as the second parameter, you can launch it like this:

./bin/tracking ../videos/practise_26_02_2013/seq_calibration/videos/cam_0.avi ./



You can switch between views using the keys 1, 2, 3, 4, 5
1 : normal view
2 : HSV view
3 : red and green mask
4 : red mask
5 : green mask
r : start/stop the recording
s : show/hide the path of the points already recorded
w : write the points to the file
d : empty the recorded points
j : delete the last recorded red point
k : delete the last recorded green point


You can use the left click to set approximately the last position of the red ball and the right click for the green ball, it's used to set the start position if possible.


You can use the playback controls to navigate in the video, you can use the space bar to pause.

Usage for getting the tracking position:
    go where you want to start the recording using the controls.
    adjust if needed the parameter to have the tracking track the ball.
    left/right click near the balls to adjust the tracking.
    When you are good you can start the recording with the key 'r' and use the spacebar to restart the video if needed.
    When the tracking circle becomes blue it means that the circle has been added to the capture.
    You can use the key 's' to see the recorded points.
    When you are happy with the result you can stop the capture with 'r' and press 'w' to write the capture to the files.
    If you are not you can use d to empty the captured points and start again somewhere else in the file.


=================

The second one is a tool to retrieve the intrinsic parameters of a camera using a calibration grid.
It's a modification of this tool: http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html

Basically you use an xml config file to configure it for each camera (you can find example in the directory xml_config).

Then you can launch it like this:

./bin/gridcalibration xml_config/cam0.xml

And when the program is running you have to press "g" to start the capture and it will give you the parameters in the output xml file.


=================

The last program is use to do the calibration from the data of the two previous tools.

Once you have all the datas, you can launch it like this:

./bin/computecalibration 5 calibration_files/ tracking_result/

Where 5 is the number of cameras, calibration_files/ is the directory that contains the parameters of the camera and tracking_result/ contains the tracking of the marker.

It will output you the final N camera projection matrices.


