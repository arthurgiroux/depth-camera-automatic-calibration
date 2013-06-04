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

	namedWindow("undistorted", 0);
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
