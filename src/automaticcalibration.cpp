#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;


Mat frame, hsv, maskredballup, maskredballdown, maskgreenballup, maskgreenballdown, mask;

static void onMouse( int event, int x, int y, int, void* )
{
	if( event != CV_EVENT_LBUTTONDOWN )
		return;

	Vec3b color = hsv.at<Vec3b>(x, y);
	cout << (int) color[0] << "\n";
	cout <<  (int)  color[1] << "\n";
	cout <<  (int) color[2] << "\n\n";
}

double distanceBetweenPoints(Vec3f p1, Vec3f p2) {
	return sqrt(pow((double) (p1[0]-p2[0]), 2.0) + pow((double) (p1[1] - p2[1]), 2.0));
}

Vec3f* findClosestPoint(vector<Vec3f> firstVector, vector<Vec3f> secondVector) {
	double mindistance = INFINITY;
	Vec3f* closest = new Vec3f[2];
	for (size_t i = 0; i < firstVector.size(); i++) {
		for (size_t j = 0; j < secondVector.size(); j++) {
			double curDist = distanceBetweenPoints(firstVector[i], secondVector[j]);
			if (curDist < mindistance) {
				mindistance = curDist;
				closest[0] = firstVector[i];
				closest[1] = secondVector[j];
			}
		}
	}
	return closest;
}


void drawCircleFromPoint(Vec3f p, Mat frame) {
	Point center(cvRound(p[0]), cvRound(p[1]));
	int radius = cvRound(p[2]);
	// draw the circle center
	circle(frame, center, 3, Scalar(0,255,0), -1, 8, 0);
	// draw the circle outline
	circle(frame, center, radius, Scalar(0,0,255), 3, 8, 0);
}

int main(int argc, char** argv) {
	if (argc < 2) {
		printf("you need to specify a file");
		return -1;
	}
	
	VideoCapture cap(argv[1]);
	if(!cap.isOpened())
		return -1;

	namedWindow("automatic calibration", 0);
	setMouseCallback( "automatic calibration", onMouse, 0 );
	bool init = false;
	
	int param1 = 20;
	int param2 = 20;
	int minradius = 5;
	int maxradius = 25;
	createTrackbar("canny threshold", "automatic calibration", &param1, 400);
	createTrackbar("center threshold", "automatic calibration", &param2, 400);
	createTrackbar("min radius", "automatic calibration", &minradius, 100);
	createTrackbar("max radius", "automatic calibration", &maxradius, 200);
	
	while (1)
	{
		Mat* toshow;
		
		if (!init) {
			toshow = &frame;
			init = true;
		}
		
		cap >> frame;
		cvtColor(frame, hsv, CV_BGR2HSV);
		// RED
		inRange(hsv, Scalar(165, 50, 50), Scalar(180, 255, 255), maskredballup);
		inRange(hsv, Scalar(0, 50, 50), Scalar(10, 255, 255), maskredballdown);
		maskredballup |= maskredballdown;
		erode(maskredballup, maskredballup, Mat());
		dilate(maskredballup, maskredballup, Mat());
		dilate(maskredballup, maskredballup, Mat());
		erode(maskredballup, maskredballup, Mat());
		GaussianBlur(maskredballup, maskredballup, Size(11, 11), 2, 2);
		vector<Vec3f> redcircles;
		// void HoughCircles(Mat& image, vector<Vec3f>& circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0)
		HoughCircles(maskredballup, redcircles, CV_HOUGH_GRADIENT, 3, 20, (param1 > 0) ? param1 : 1, (param2 > 0) ? param2 : 1, (minradius > 0) ? minradius : 1, (maxradius > 0) ? maxradius : 1);

		// GREEN
		inRange(hsv, Scalar(140, 50, 50), Scalar(160, 255, 255), maskgreenballup);
		inRange(hsv, Scalar(30, 30, 50), Scalar(60, 255, 255), maskgreenballdown);
		maskgreenballup |= maskgreenballdown;
		erode(maskgreenballup, maskgreenballup, Mat());
		dilate(maskgreenballup, maskgreenballup, Mat());
		dilate(maskgreenballup, maskgreenballup, Mat());
		erode(maskgreenballup, maskgreenballup, Mat());
		GaussianBlur(maskgreenballup, maskgreenballup, Size(11, 11), 2, 2);
		vector<Vec3f> greencircles;

		mask = maskgreenballup | maskredballup;
		// void HoughCircles(Mat& image, vector<Vec3f>& circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0)
		HoughCircles(maskgreenballup, greencircles, CV_HOUGH_GRADIENT, 3, 20, (param1 > 0) ? param1 : 1, (param2 > 0) ? param2 : 1, (minradius > 0) ? minradius : 1, (maxradius > 0) ? maxradius : 1);

		/*for (size_t i = 0; i < redcircles.size(); i++)
		{
			Point center(cvRound(redcircles[i][0]), cvRound(redcircles[i][1]));
			int radius = cvRound(redcircles[i][2]);
			// draw the circle center
			circle( frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
			// draw the circle outline
			circle( frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
		}

		for( size_t i = 0; i < redcircles.size(); i++ )
		{
			Point center(cvRound(redcircles[i][0]), cvRound(redcircles[i][1]));
			int radius = cvRound(redcircles[i][2]);
			// draw the circle center
			circle( frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
			// draw the circle outline
			circle( frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
		}*/

		Vec3f* closest = findClosestPoint(redcircles, greencircles);
		cout << closest[0][0] << " " << closest[0][1] << endl;
		drawCircleFromPoint(closest[0], frame);
		drawCircleFromPoint(closest[1], frame);

		imshow("automatic calibration", *toshow);
		
		int key = waitKey(30);
		switch (key) {

			case 49:
				toshow = &frame;
				break;
				
			case 50:
				toshow = &hsv;
				break;
				
			case 51:
				toshow = &mask;
				break;

			case 52:
				toshow = &maskredballup;
				break;

			case 53:
				toshow = &maskgreenballup;
				break;
				
			case 27: return 0;
				
			default:
				break;
		}
	}

	return 0;
}
