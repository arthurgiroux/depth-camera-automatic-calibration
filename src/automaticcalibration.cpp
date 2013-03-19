#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;


Mat frame, hsv, mask, mask1, mask2;

static void onMouse( int event, int x, int y, int, void* )
{
    if( event != CV_EVENT_LBUTTONDOWN )
		return;
	
	Vec3b color = hsv.at<Vec3b>(x, y);
	cout << (int) color[0] << "\n";
	cout <<  (int)  color[1] << "\n";
	cout <<  (int) color[2] << "\n\n";
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
		/*inRange(hsv, Scalar(165, 50, 50), Scalar(180, 255, 255), mask1);
		inRange(hsv, Scalar(0, 50, 50), Scalar(10, 255, 255), mask2);*/
		// GREEN
		inRange(hsv, Scalar(165, 50, 50), Scalar(180, 255, 255), mask1);
		inRange(hsv, Scalar(0, 50, 50), Scalar(10, 255, 255), mask2);
		mask = mask1 | mask2;
		//inRange(hsv, Scalar(hue_min, saturation_min, value_min), Scalar(hue_max, saturation_max, value_max), mask);
		erode(mask, mask, Mat());
		dilate(mask, mask, Mat());
		dilate(mask, mask, Mat());
		erode(mask, mask, Mat());
		GaussianBlur(mask, mask, Size(11, 11), 2, 2);
		vector<Vec3f> circles;
		// void HoughCircles(Mat& image, vector<Vec3f>& circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0)
		HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 3, 20, (param1 > 0) ? param1 : 1, (param2 > 0) ? param2 : 1, (minradius > 0) ? minradius : 1, (maxradius > 0) ? maxradius : 1);
    for( size_t i = 0; i < circles.size(); i++ )
    {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         circle( frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( frame, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }
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
				
			case 27: return 0;
				
			default:
				break;
    	}
	}

	return 0;
}
