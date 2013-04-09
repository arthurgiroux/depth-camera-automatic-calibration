#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio>
#include <string>
#include <iostream>

#define MAX_STICK_LENGTH 100
#define ITERATION_PER_FRAME 3

#define COLOR_RED Scalar(0, 0, 255)
#define COLOR_GREEN Scalar(0, 255, 0)
#define COLOR_BLUE Scalar(255, 0, 0)
#define COLOR_YELLOW Scalar(0, 255, 255)
#define COLOR_TEAL Scalar(255, 255, 0)
#define COLOR_PURPLE Scalar(255, 0, 255)

#define KEY_ESC 27

using namespace cv;
using namespace std;

static void onMouse( int event, int x, int y, int, void* data )
{
	if (event == CV_EVENT_LBUTTONDOWN) {

		/*Vec3b color = hsv.at<Vec3b>(x, y);
		cout << (int) color[0] << endl;
		cout <<  (int)  color[1] << endl;
		cout <<  (int) color[2] << endl << endl;*/
		//cout << "(" << x << ", " << y << ")" << endl;
		
		*(((Vec3f[]) data)[0]) = Vec3f(x, y, 0);
	} else if (event == CV_EVENT_RBUTTONDOWN) {
		*(((Vec3f[]) data)[1]) = Vec3f(x, y, 0);
	}
}

double distanceBetweenPoints(Vec3f p1, Vec3f p2) {
	// sqrt((x1 - x2)^2 + (y1 - y2)^2)
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1]));
}


Vec3f findClosestPoint(const vector<Vec3f>* points, Vec3f lastKnownPoint) {
	double mindistance = INFINITY;
	int index = -1;
	for (size_t i = 0; i < points->size(); i++) {
		double curDist = distanceBetweenPoints((*points)[i], lastKnownPoint);
		if (curDist < mindistance) {
			mindistance = curDist;
			index = i;
		}
	}
	return (*points)[index];
}

void drawCircleFromPoint(Vec3f p, Mat* frame, Scalar color = COLOR_RED) {
	Point center(p[0], p[1]);
	// draw the circle center
	circle(*frame, center, 3, COLOR_GREEN, -1);
	// draw the circle outline
	circle(*frame, center, p[2], color, 3);
}


bool recordPositionOfBall(vector<pair<int, Vec3f> >* records, Vec3f bestCandidate,
                          Vec3f* lastKnownPoint, int currentFrameNumber) {
	if (records->size() > 0) {
		pair<int, Vec3f> lastRegistered = records->back();
			//cout << distanceBetweenPoints(lastRegistered.second, currentRed) << " --- " << (curFrame - lastRegistered.first)  << endl;
		if (*lastKnownPoint != lastRegistered.second ||
		    distanceBetweenPoints(lastRegistered.second, bestCandidate) <= (ITERATION_PER_FRAME * (currentFrameNumber - lastRegistered.first))) {
			*lastKnownPoint = bestCandidate;
			records->push_back(make_pair(currentFrameNumber, bestCandidate));
			return true;
		}
	}
	else {
		*lastKnownPoint = bestCandidate;
		records->push_back(make_pair(currentFrameNumber, bestCandidate));
		return true;
	}

	return false;
}


void writeVectorsToFile(const string filename,  const vector<pair<int, Vec3f> >* records) {
	ofstream file(filename);
	if (file.is_open()) {
		for(size_t i = 0; i < records->size(); i++) {
			pair<int, Vec3f> point = (*records)[i];
			file << point.first << " " << point.second[0] << " " << point.second[1] << " " << point.second[2] << endl;
		}
		file.close();
	}
}

void showPath(Mat* frame, const vector<pair<int, Vec3f> >* records, Scalar color) {
	for(size_t i = 0; i < records->size(); i++) {
		pair<int, Vec3f> point = (*records)[i];
		Point center(point.second[0], point.second[1]);
		circle(*frame, center, 1, color, -1);
	}
}

void OnChangePosition(int value, void* data) {
	if (value >= 0) {
		(*(VideoCapture*) data).set(CV_CAP_PROP_POS_MSEC, 1000 * value);
	}
}

int main(int argc, char** argv) {
	if (argc < 3) {
		printf("usage: ./" + argv[0] + " path-to-video output-directory");
		return -1;
	}
	
	VideoCapture cap(argv[1]);
	if(!cap.isOpened())
		return -1;


	Mat frame, hsv, maskredballup, maskredballdown, maskgreenballup, maskgreenballdown, mask;
	vector<pair<int, Vec3f> > pointsRed;
	vector<pair<int, Vec3f> > pointsGreen;
	Vec3f lastRed, lastGreen;
	Vec3f[] lastPair = { &lastRed, &lastGreen };

	int currentFrameNumber = 0;


	bool paused = false;
	bool recording = false;
	bool showpath = false;
	bool somethingToRead = true;

	namedWindow("automatic calibration", 0);
	setMouseCallback("automatic calibration", onMouse, lastPair);
	bool init = false;
	
	namedWindow("parameters", 0);
	int param1 = 40;
	int param2 = 10;
	int minradius = 1;
	int maxradius = 20;

	createTrackbar("canny threshold", "parameters", &param1, 400);
	createTrackbar("center threshold", "parameters", &param2, 400);
	createTrackbar("min radius", "parameters", &minradius, 100);
	createTrackbar("max radius", "parameters", &maxradius, 200);
	

	// PLAYBACK CONTROLS
	namedWindow("playback controls", 0);
	createTrackbar("position", "playback controls", 0, floor(cap.get(CV_CAP_PROP_FRAME_COUNT) / cap.get(CV_CAP_PROP_FPS)), OnChangePosition, (void*) &cap);

	while (1)
	{
		Mat* toshow;
		
		if (!init) {
			toshow = &frame;
			init = true;
		}

		if (!paused) {
			somethingToRead = cap.grab();
		}
		if (somethingToRead) {
			cap.retrieve(frame, 0);
		}

		currentFrameNumber = cap.get(CV_CAP_PROP_POS_FRAMES);

		setTrackbarPos("position", "playback controls", floor(currentFrameNumber / cap.get(CV_CAP_PROP_FPS)));
		cvtColor(frame, hsv, CV_BGR2HSV);

		// RED
		inRange(hsv, Scalar(165, 50, 50), Scalar(180, 255, 255), maskredballup);
		inRange(hsv, Scalar(0, 50, 50), Scalar(10, 255, 255), maskredballdown);
		maskredballup |= maskredballdown;
		morphologyEx(maskredballup, maskredballup, MORPH_OPEN, Mat());
		morphologyEx(maskredballup, maskredballup, MORPH_CLOSE, Mat());
		medianBlur(maskredballup, maskredballup, 7);
		vector<Vec3f> redcircles;

		// void HoughCircles(Mat& image, vector<Vec3f>& circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0)
		HoughCircles(maskredballup, redcircles, CV_HOUGH_GRADIENT, 2, 20, (param1 > 0) ? param1 : 1,
		             (param2 > 0) ? param2 : 1, (minradius > 0) ? minradius : 1, (maxradius > 0) ? maxradius : 1);

		// GREEN
		inRange(hsv, Scalar(140, 50, 50), Scalar(160, 255, 255), maskgreenballup);
		inRange(hsv, Scalar(30, 30, 50), Scalar(60, 255, 255), maskgreenballdown);
		maskgreenballup |= maskgreenballdown;
		//GaussianBlur(maskgreenballup, maskgreenballup, Size(15, 15), 2, 2);
		morphologyEx(maskgreenballup, maskgreenballup, MORPH_OPEN, Mat());
		morphologyEx(maskgreenballup, maskgreenballup, MORPH_CLOSE, Mat());
		medianBlur(maskgreenballup, maskgreenballup, 7);
		vector<Vec3f> greencircles;

		mask = maskgreenballup | maskredballup;

		// void HoughCircles(Mat& image, vector<Vec3f>& circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0)
		HoughCircles(maskgreenballup, greencircles, CV_HOUGH_GRADIENT, 2, 20, (param1 > 0) ? param1 : 1,
		             (param2 > 0) ? param2 : 1, (minradius > 0) ? minradius : 1, (maxradius > 0) ? maxradius : 1);


		if (redcircles.size() > 0) {
			circle(frame, Point(lastRed[0], lastRed[1]), 3, COLOR_TEAL, -1);
			Vec3f bestCandidate = findClosestPoint(&redcircles, lastRed);
			Scalar color = COLOR_RED;

			if (recording && !paused) {
				if (recordPositionOfBall(&pointsRed, bestCandidate, &lastRed, currentFrameNumber)) {
					color = COLOR_BLUE;
				}
			}
			drawCircleFromPoint(bestCandidate, &frame, color);
		}

		if (greencircles.size() > 0) {
			circle(frame, Point(lastGreen[0], lastGreen[1]), 3, COLOR_PURPLE, -1);
			Vec3f bestCandidate = findClosestPoint(&greencircles, lastGreen);
			Scalar color = COLOR_RED;

			if (recording && !paused) {
				if (recordPositionOfBall(&pointsGreen, bestCandidate, &lastGreen, currentFrameNumber)) {
					color = COLOR_BLUE;
				}
			}
			drawCircleFromPoint(bestCandidate, &frame, color);
		}

		if (showpath) {
			showPath(&frame, &pointsRed, COLOR_TEAL);
			showPath(&frame, &pointsGreen, COLOR_PURPLE);
		}

		imshow("automatic calibration", *toshow);

		int key = waitKey(30);
		switch (key) {

			case '1':
				toshow = &frame;
				break;

			case '2':
				toshow = &hsv;
				break;

			case '3':
				toshow = &mask;
				break;

			case '4':
				toshow = &maskredballup;
				break;

			case '5':
				toshow = &maskgreenballup;
				break;

			case 'r':
				recording = !recording;
				break;

			case 'w':
				//cout << substr()
				//writeVectorsToFile(argv[2] + "/outpu.txt");
				break;

			case 's':
				showpath = !showpath;
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
