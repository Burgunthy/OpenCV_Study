#define _CRT_SECURE_NO_WARNINGS

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include <sstream>
#include <iostream>  
#include <string> 

using namespace cv;
using namespace std;

//* Camera Calbration Variable -- New One */
// intrinsic parameter --- Left
const double fxL = 531.48547; const double fyL = 535.38115;
const double cxL = 317.36588; const double cyL = 231.51589;
// extrinsic paramter --- Left
const double k1L = 0.10394; const double k2L = -0.31141; const double p1L = -0.01591; const double p2L = 0.00107; const double k3L = 0.0;

// intrinsic parameter --- Right
const double fxR = 528.39448; const double fyR = 532.66379;
const double cxR = 325.58373; const double cyR = 221.11119;
// extrinsic paramter --- Right
const double k1R = 0.10519; const double k2R = -0.20104; const double p1R = -0.01859; const double p2R = -0.00516; const double k3R = 0.0;

// Translation, Rotation
const double trs1 = -42.75357; const double trs2 = -0.05412; const double trs3 = -0.15031;
const double rot_om1 = -0.00638; const double rot_om2 = 0.02294; const double rot_om3 = -0.00034;


VideoCapture * capture;

Mat ori;
Mat img1, img2;
Mat disp1, disp2;

int frameWidth = 640;
int frameHeight = 480;

// Create calibration Matrix
cv::Mat intrinsicL = cv::Mat(3, 3, CV_64FC1, cvScalar(0.));
cv::Mat intrinsicR = cv::Mat(3, 3, CV_64FC1, cvScalar(0.));
cv::Mat Translation = cv::Mat(3, 1, CV_64FC1, cvScalar(0.));
cv::Mat om = cv::Mat(1, 3, CV_64FC1, cvScalar(0.));
cv::Mat Rotation = cv::Mat(3, 3, CV_64FC1, cvScalar(0.));

cv::Mat distCoeffL = cv::Mat(5, 1, CV_64FC1, cvScalar(0.));
cv::Mat distCoeffR = cv::Mat(5, 1, CV_64FC1, cvScalar(0.));

cv::Mat imgDistL = cv::Mat(frameHeight, frameWidth, CV_8UC3, cvScalar(0.));	// well, we gonna use 480*640(VGA) frame
cv::Mat imgDistR = cv::Mat(frameHeight, frameWidth, CV_8UC3, cvScalar(0.));
cv::Mat imgRectiL = cv::Mat(frameHeight, frameWidth, CV_8UC3, cvScalar(0.));
cv::Mat imgRectiR = cv::Mat(frameHeight, frameWidth, CV_8UC3, cvScalar(0.));

// stereoRectify variable & variable for result frame
cv::Mat R1, R2, P1, P2, Q;
cv::Mat map1x, map1y, map2x, map2y;
cv::Mat rectResultLeft, rectResultRight;
cv::Mat RIden = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
cv::Mat rectResultFrame;

int main() {

	distCoeffL = (Mat_<double>(5, 1) << k1L, k2L, p1L, p2L, k3L);
	distCoeffR = (Mat_<double>(5, 1) << k1R, k2R, p1R, p2R, k3R);
	intrinsicL = (Mat_<double>(3, 3) << fxL, 0, cxL, 0, fyL, cyL, 0, 0, 1);
	intrinsicR = (Mat_<double>(3, 3) << fxR, 0, cxR, 0, fyR, cyR, 0, 0, 1);
	Translation = (Mat_<double>(3, 1) << trs1, trs2, trs3);
	om = (Mat_<double>(1, 3) << rot_om1, rot_om2, rot_om3);
	Rodrigues(om, Rotation);

	stereoRectify(intrinsicL, distCoeffL, intrinsicR, distCoeffR, cv::Size(frameWidth, frameHeight), Rotation, Translation, R1, R2, P1, P2, Q);

	capture = new cv::VideoCapture(1);

	if (!capture) {
		printf("Could not capture a camera\n\7");
		return -1;
	}

	char filename1[200];
	char filename2[200];

	int i = 0;

	Ptr<StereoBM> bm = StereoBM::create(32, 11);
	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 48, 11);

	while (1) {

		capture->read(ori);

		img2 = ori(cv::Rect(frameWidth, 0, frameWidth, frameHeight)).clone();
		ori = ori(cv::Rect(0, 0, frameWidth, frameHeight)).clone();


		cvtColor(ori, img1, CV_BGR2GRAY);
		cvtColor(img2, img2, CV_BGR2GRAY);

		initUndistortRectifyMap(intrinsicL, distCoeffL, R1, P1, img1.size(), CV_8U, map1x, map1y);
		initUndistortRectifyMap(intrinsicR, distCoeffR, R2, P2, img2.size(), CV_8U, map2x, map2y);
		remap(img1, rectResultLeft, map1x, map1y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
		remap(img2, rectResultRight, map2x, map2y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

		//bm->compute(img1, img2, disp1);
		sgbm->compute(img1, img2, disp2);

		//reprojectImageTo3D(disp2, disp2, Q, false, CV_16S);

		//cout << disp2.cols << " " << disp2.rows << " " << disp2.channels() << " " << disp2.depth() << " " << disp2.type() << endl;

		// normalize(disp1, disp1, 0, 255, CV_MINMAX, CV_8U);
		//normalize(disp2, disp2, 0, 255, CV_MINMAX, CV_8U);

		imshow("left", ori);

		//imshow("bm", disp1);
		// disp2.convertTo(disp2, CV_16SC1);
		disp2.convertTo(disp2, CV_16UC1);
		for (int y = 0; y < frameHeight; y++) {
			for (int x = 0; x < frameWidth; x++) {
				if (disp2.at<ushort>(y, x) != 0)
					disp2.at<ushort>(y, x) = (ushort)(531 * 60 / disp2.at<ushort>(y, x));
			}
		}

		//normalize(disp2, disp2, 0, 255, CV_MINMAX, CV_8U);

		imshow("sgbm", disp2);

		sprintf(filename1, "./stereo/img_%04d.png", i);
		sprintf(filename2, "./stereo/depth_%04d.png", i);

		imwrite(filename1, ori);
		imwrite(filename2, disp2);

		cout << "iteration : " << i << endl;
		i++;

		if (waitKey(27) == 27) break;
	}


	cout << disp1.type() << endl;

	return 0;
}