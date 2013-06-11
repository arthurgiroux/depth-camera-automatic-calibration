/*
Copyright (C) 2013 Giroux Arthur (arthur.giroux@epfl.ch)

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <fstream>

// Maximum stick length in pixel
#define MAX_STICK_LENGTH 100
// Maximum number in pixel (per frame elapsed) for the distance between the previous marker and the new one
#define ITERATION_PER_FRAME 20

#define COLOR_RED Scalar(0, 0, 255)
#define COLOR_GREEN Scalar(0, 255, 0)
#define COLOR_BLUE Scalar(255, 0, 0)
#define COLOR_YELLOW Scalar(0, 255, 255)
#define COLOR_TEAL Scalar(255, 255, 0)
#define COLOR_PURPLE Scalar(255, 0, 255)

#define RED_SEGMENTATION_VALUE_BRIGHT_UP Scalar(180, 255, 255)
#define RED_SEGMENTATION_VALUE_BRIGHT_DOWN Scalar(165, 50, 50)

#define RED_SEGMENTATION_VALUE_DARK_UP Scalar(10, 255, 255)
#define RED_SEGMENTATION_VALUE_DARK_DOWN Scalar(0, 50, 50)

#define GREEN_SEGMENTATION_VALUE_BRIGHT_UP Scalar(160, 255, 255)
#define GREEN_SEGMENTATION_VALUE_BRIGHT_DOWN Scalar(140, 50, 50)

#define GREEN_SEGMENTATION_VALUE_DARK_UP Scalar(80, 255, 255)
#define GREEN_SEGMENTATION_VALUE_DARK_DOWN Scalar(20, 50, 50)

// Keycode for the escape key
#define KEY_ESC 27

using namespace cv;
using namespace std;

/*
	Callback function to handle the mouse event;
	data is a pair of pointers to the position of the old last known marker position
	
*/
static void onMouse(int event, int x, int y, int, void* data)
{
	// Left click handles the red marker
	if (event == CV_EVENT_LBUTTONDOWN) {
		pair<Vec3f*, Vec3f*> *points = static_cast<pair<Vec3f*, Vec3f*>*>(data);
		*points->first = Vec3f(x, y, 0);

	// Right click handles the green marker
	} else if (event == CV_EVENT_RBUTTONDOWN) {
		pair<Vec3f*, Vec3f*> *points = static_cast<pair<Vec3f*, Vec3f*>*>(data);
		*points->second = Vec3f(x, y, 0);
	}
}

/*
	Return the distance in pixel between two 2d point.
	It uses a Vec3f and not a Vec2f because it's used right after the hough circle transformation
	where we get 3 coordinates, the last one being the radius.
*/
double distanceBetweenPoints(Vec3f p1, Vec3f p2) {
	// sqrt((x1 - x2)^2 + (y1 - y2)^2)
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1]));
}


/*
	Find the the closest points between the last known marker and the set of detected circle (non-empty)
*/
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

/*
	Draw the center and a circle into the image frame at the position p with the given color
*/
void drawCircleFromPoint(Vec3f p, Mat* frame, Scalar color = COLOR_RED) {
	Point center(p[0], p[1]);
	// draw the circle center
	circle(*frame, center, 3, COLOR_GREEN, -1);
	// draw the circle outline
	circle(*frame, center, p[2], color, 3);
}


/*
	Record or not the new marker position given the set of recorded marker position,
	the best candidate (closest point to the previous marker position),
	The last known point (might have been set manually),
	and the current frame number
*/
bool recordPositionOfBall(vector<pair<int, Vec3f> >* records, Vec3f bestCandidate,
                          Vec3f* lastKnownPoint, int currentFrameNumber) {
	// If we have already recorded something we check the distance
	if (records->size() > 0) {
		pair<int, Vec3f> lastRegistered = records->back();

		// We check that the best candidate is not too far away from the last known position
		// We allow it to be further away depending on the number of frame elapsed
		if (distanceBetweenPoints(*lastKnownPoint, bestCandidate) <= (ITERATION_PER_FRAME * abs(currentFrameNumber - lastRegistered.first))) {
			*lastKnownPoint = bestCandidate;
			records->push_back(make_pair(currentFrameNumber, bestCandidate));
			return true;
		}
	}
	// If we haven't recorded anything yet we just record the best candidate
	else {
		*lastKnownPoint = bestCandidate;
		records->push_back(make_pair(currentFrameNumber, bestCandidate));
		return true;
	}

	return false;
}

/*
	Write the recorded marker positions to path/filename_suffix.txt
	The format is:
		x y radius
*/
void writeVectorsToFile(const string filename, const string suffix, const string path, const vector<pair<int, Vec3f> >* records) {

	string finalname;

	// Get the filename
	size_t posName = filename.find_last_of("/");
	size_t posExt = filename.find_last_of(".");
	if (posName == std::string::npos) {
		posName = 0;
	}
	finalname = filename.substr(posName, posExt - posName);

	finalname = path + "/" + finalname + "_" + suffix + ".txt";

	ofstream file(finalname.c_str());
	if (file.is_open()) {
		for(size_t i = 0; i < records->size(); i++) {
			pair<int, Vec3f> point = (*records)[i];
			file << point.first << " " << point.second[0] << " " << point.second[1] << " " << point.second[2] << endl;
		}
		file.close();
	}

	cout << finalname << " written." << endl;
}

/*
	Show all the recorded marker positions into the frame with the given color
*/
void showPath(Mat* frame, const vector<pair<int, Vec3f> >* records, Scalar color) {
	for(size_t i = 0; i < records->size(); i++) {
		pair<int, Vec3f> point = (*records)[i];
		Point center(point.second[0], point.second[1]);
		circle(*frame, center, 1, color, -1);
	}
}

/*
	Callback function to navigate into the video stream
*/
void OnChangePosition(int value, void* data) {
	if (value >= 0) {
		(*(VideoCapture*) data).set(CV_CAP_PROP_POS_MSEC, 1000 * value);
	}
}

int main(int argc, char** argv) {

	bool disable_navigation = false;
	if (argc < 3) {
		cout << "usage: " << argv[0] << " [-disable-navigation] <path-to-video-file> <output-directory>" << endl;
		return -1;
	}

	if (strcmp(argv[1], "-disable-navigation") == 0) {
		if (argc < 4) {
			cout << "usage: " << argv[0] << " [-disable-navigation] <path-to-video-file> <output-directory>" << endl;
			return -1;
		}
		else {
			disable_navigation = true;
			argv++;
		}
	}
	
	// We open the video file
	VideoCapture cap(argv[1]);
	if(!cap.isOpened())
		return -1;


	Mat frame, hsv, maskredballup, maskredballdown, maskgreenballup, maskgreenballdown, mask;

	// the vectors of pair where the key is the frame number and the value is the marker position
	vector<pair<int, Vec3f> > pointsRed;
	vector<pair<int, Vec3f> > pointsGreen;
	// last known position of the markers
	Vec3f lastRed, lastGreen;

	int currentFrameNumber = 0;


	bool paused = false;
	bool recording = false;
	bool showpath = false;
	bool somethingToRead = true;

	namedWindow("automatic calibration");
	pair<Vec3f*, Vec3f*> callbackarg = make_pair(&lastRed, &lastGreen);
	setMouseCallback("automatic calibration", onMouse, &callbackarg);
	bool init = false;
	
	namedWindow("parameters", 0);
	// Initial parameters for Hough Circle Transform
	int param1 = 60;
	int param2 = 10;
	int minradius = 1;
	int maxradius = 20;
	int mindist = 30;
	int dp = 1;

	createTrackbar("canny threshold", "parameters", &param1, 400);
	createTrackbar("center threshold", "parameters", &param2, 400);
	createTrackbar("min radius", "parameters", &minradius, 100);
	createTrackbar("max radius", "parameters", &maxradius, 200);
	createTrackbar("min distance", "parameters", &mindist, 100);
	createTrackbar("accumulator resolution", "parameters", &dp, 10);
	

	// Playback controls
	if (!disable_navigation) {
		namedWindow("playback controls", 0);
		createTrackbar("position", "playback controls", 0, floor(cap.get(CV_CAP_PROP_FRAME_COUNT) / cap.get(CV_CAP_PROP_FPS)), OnChangePosition, (void*) &cap);
	}

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
			// We set the last red and green at the middle of the screen
			lastGreen = lastRed = Vec3f(frame.cols/2.0, frame.rows/2.0);
		}

		currentFrameNumber = cap.get(CV_CAP_PROP_POS_FRAMES);

		// We draw some information on the top-left corner of the image
		std::stringstream ss;
		ss << "frame : " << currentFrameNumber << " captured : " << pointsRed.size() << " red, " << pointsGreen.size() << " green"; 
		putText(frame, ss.str(), Point(10, 50), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255));
		if (!disable_navigation) {
			setTrackbarPos("position", "playback controls", floor(currentFrameNumber / cap.get(CV_CAP_PROP_FPS)));
		}

		// We go in HSV color space
		cvtColor(frame, hsv, CV_BGR2HSV);

		// We do the red segmentation
		inRange(hsv, RED_SEGMENTATION_VALUE_BRIGHT_DOWN, RED_SEGMENTATION_VALUE_BRIGHT_UP, maskredballup);
		inRange(hsv, RED_SEGMENTATION_VALUE_DARK_DOWN, RED_SEGMENTATION_VALUE_DARK_UP, maskredballdown);
		maskredballup |= maskredballdown;

		// We do an opening and a closing transformation to get rid of the noise
		morphologyEx(maskredballup, maskredballup, MORPH_OPEN, Mat());
		morphologyEx(maskredballup, maskredballup, MORPH_CLOSE, Mat());

		// Median blur to get rid of the last imperfections and smooth the edges
		medianBlur(maskredballup, maskredballup, 7);


		vector<Vec3f> redcircles;

		Canny(maskredballup, maskredballup, 100, 200);

		// Do the circle detection and get the result into redcircles
		HoughCircles(maskredballup, redcircles, CV_HOUGH_GRADIENT, (dp > 0) ? dp : 1, (mindist > 0) ? mindist : 1, (param1 > 0) ? param1 : 1,
		             (param2 > 0) ? param2 : 1, (minradius > 0) ? minradius : 1, (maxradius > 0) ? maxradius : 1);

		// We do the green segmentation
		inRange(hsv, GREEN_SEGMENTATION_VALUE_BRIGHT_DOWN, GREEN_SEGMENTATION_VALUE_BRIGHT_UP, maskgreenballup);
		inRange(hsv, GREEN_SEGMENTATION_VALUE_DARK_DOWN, GREEN_SEGMENTATION_VALUE_DARK_UP, maskgreenballdown);
		maskgreenballup |= maskgreenballdown;

		morphologyEx(maskgreenballup, maskgreenballup, MORPH_OPEN, Mat());
		morphologyEx(maskgreenballup, maskgreenballup, MORPH_CLOSE, Mat());

		medianBlur(maskgreenballup, maskgreenballup, 7);

		vector<Vec3f> greencircles;

		mask = maskgreenballup | maskredballup;

		Canny(maskgreenballup, maskgreenballup, 100, 200);

		HoughCircles(maskgreenballup, greencircles, CV_HOUGH_GRADIENT, (dp > 0) ? dp : 1, (mindist > 0) ? mindist : 1,  (param1 > 0) ? param1 : 1,
		             (param2 > 0) ? param2 : 1, (minradius > 0) ? minradius : 1, (maxradius > 0) ? maxradius : 1);



		// We draw the last red marker position
		circle(frame, Point(lastRed[0], lastRed[1]), 3, COLOR_TEAL, -1);

		// If we have detected some circles we get the closest one and we record the position if needed
		if (redcircles.size() > 0) {
			Vec3f bestCandidate = findClosestPoint(&redcircles, lastRed);
			Scalar color = COLOR_RED;

			if (recording && !paused) {
				if (recordPositionOfBall(&pointsRed, bestCandidate, &lastRed, currentFrameNumber)) {
					color = COLOR_BLUE;
				}
			}
			drawCircleFromPoint(bestCandidate, &frame, color);
		}

		circle(frame, Point(lastGreen[0], lastGreen[1]), 3, COLOR_PURPLE, -1);
		if (greencircles.size() > 0) {
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
		// modulus to get rid of a bug in ubuntu
		switch (key % 256) {

			case '1': // Original frame
				toshow = &frame;
				break;

			case '2': // HSV frame
				toshow = &hsv;
				break;

			case '3': // Red AND Green segmentation
				toshow = &mask;
				break;

			case '4': // Red segmentation
				toshow = &maskredballup;
				break;

			case '5': // Green segmentation
				toshow = &maskgreenballup;
				break;

			case 'r': // Start / Stop the recording
				recording = !recording;
				break;

			case 'w': // Write the marker positions to file
				writeVectorsToFile(string(argv[1]), "red", string(argv[2]), &pointsRed);
				writeVectorsToFile(string(argv[1]), "green", string(argv[2]), &pointsGreen);
				break;

			case 's': // Show / Hide the path
				showpath = !showpath;
				break;

			case 'd': // Delete all the recorded position
				pointsRed.clear();
				pointsGreen.clear();
				break;

			case 'j': // Delete the last red marker position
				pointsRed.pop_back();
				break;

			case 'k':  // Delete the last green marker position
				pointsGreen.pop_back();
				break;

			case KEY_ESC: // Close the window
				return 0;

			case ' ': // Pause / Unpause
				paused = !paused;
				break;

			default:
				break;

		}
	}
	return 0;
}
