depth-camera-automatic-calibration
==================================

Automatic calibration tool for depth and rgb camera

To build the project you may have to change the LDFLAG variable in the Makefile,
to determine the value of this variable you can use the command pkg-config --libs --cflags opencv

The program take the video input as the first parameter, you can launch it like this:

./bin/automaticcalibration ../videos/practise_26_02_2013/seq_calibration/videos/cam_0.avi


You can switch between views using the keys 1, 2, 3, 4, 5
1 : normal view
2 : HSV view
3 : red and green mask
4 : red mask
5 : green mask
