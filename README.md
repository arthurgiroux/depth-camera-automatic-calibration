depth-camera-automatic-calibration
==================================

Automatic calibration tool for depth and rgb camera

To build the project you may have to change the LDFLAG variable in the Makefile,
to determine the value of this variable you can use the command pkg-config --libs --cflags opencv

The program take the video input as the first parameter and the output directory for the calibration files as the second parameter, you can launch it like this:

./bin/automaticcalibration ../videos/practise_26_02_2013/seq_calibration/videos/cam_0.avi ./



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

