/*
Copyright (c) 2013, Giroux Arthur (arthur.giroux@epfl.ch)
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
* Neither the name of the University of California, Berkeley nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#define KEY_ESC 27

using namespace cv;
using namespace std;

void OnChangePosition(int value, void* data) {
	if (value >= 0) {
		(*(VideoCapture*) data).set(CV_CAP_PROP_POS_MSEC, 1000 * value);
	}
}

int main(int argc, char** argv) {
	if (argc < 3) {
		cout << "usage: " << argv[0] << " <path-to-video-file> <calib-config>" << endl;
		return -1;
	}

	Mat cameraMatrix, distCoeffs;
	
    FileStorage fs(argv[2], FileStorage::READ);
    if (fs.isOpened()) {
            fs["Camera_Matrix"] >> cameraMatrix;
            fs["Distortion_Coefficients"] >> distCoeffs;
            cout << "camera matrix: " << cameraMatrix << endl << "distortion coeffs: " << distCoeffs << endl;
    }

	VideoCapture cap(argv[1]);
	if(!cap.isOpened())
		return -1;


	Mat frame;
	Mat undist;

	int currentFrameNumber = 0;


	bool paused = false;
	bool init = false;
	bool somethingToRead = true;

	namedWindow("undistorted");
	// PLAYBACK CONTROLS
	namedWindow("playback controls", 0);
	createTrackbar("position", "playback controls", 0, floor(cap.get(CV_CAP_PROP_FRAME_COUNT) / cap.get(CV_CAP_PROP_FPS)), OnChangePosition, (void*) &cap);

	while (1)
	{
		Mat* toshow;

		if (!paused) {
			somethingToRead = cap.grab();
		}
		if (somethingToRead) {
			cap.retrieve(frame, 0);
		}

		if (!init) {
			toshow = &frame;
			init = true;
		}

		currentFrameNumber = cap.get(CV_CAP_PROP_POS_FRAMES);
		//setTrackbarPos("position", "playback controls", floor(currentFrameNumber / cap.get(CV_CAP_PROP_FPS)));

		undistort(frame, undist, cameraMatrix, distCoeffs);

		imshow("undistorted", *toshow);

		int key = waitKey(30);
		switch (key % 256) {

			case '1':
				toshow = &frame;
				break;

			case '2':
				toshow = &undist;
				break;

			case KEY_ESC:
				return 0;

			case ' ':
				paused = !paused;
				break;

			default:
				break;

		}
	}
	return 0;
}
