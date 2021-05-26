//#define _CRT_SECURE_NO_WARNINGS
//
//#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/calib3d/calib3d.hpp"
//
//#include <iostream>
//#include <ctype.h>
//#include <algorithm> // for copy
//#include <iterator> // for ostream_iterator
//#include <vector>
//#include <ctime>
//#include <sstream>
//#include <fstream>
//#include <string>
//
//using namespace cv;
//using namespace std;
//
//#define MAX_FRAME 1000
//#define MIN_NUM_FEAT 100
//#define MAX_Z 3800
//
//void featureTracking(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status) {
//
//	//this function automatically gets rid of points for which tracking fails
//
//	vector<float> err;
//	Size winSize = Size(21, 21);
//	TermCriteria termcrit = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);
//
//	calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);
//
//	//getting rid of points for which the KLT tracking failed or those who have gone outside the frame
//	int indexCorrection = 0;
//	for (int i = 0; i < status.size(); i++)
//	{
//		Point2f pt = points2.at(i - indexCorrection);
//		if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
//			if ((pt.x < 0) || (pt.y < 0)) {
//				status.at(i) = 0;
//			}
//			points1.erase(points1.begin() + (i - indexCorrection));
//			points2.erase(points2.begin() + (i - indexCorrection));
//			indexCorrection++;
//		}
//	}
//}
//
//void featureDetection(Mat img_1, vector<Point2f>& points1) {   //uses FAST as of now, modify parameters as necessary
//	vector<KeyPoint> keypoints_1;
//	int fast_threshold = 40;
//	bool nonmaxSuppression = true;
//	FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
//	KeyPoint::convert(keypoints_1, points1, vector<int>());
//}
//
//int gaggmain(int argc, char** argv) {
//
//	Mat img_1, img_2;
//
//	char filename1[200];
//	char filename2[200];
//	sprintf(filename1, "./realsense_missing/img_%04d.png", 0);
//	sprintf(filename2, "./realsense_missing/img_%04d.png", 1);
//
//	Mat img_1_c = imread(filename1);
//	Mat img_2_c = imread(filename2);
//
//	if (!img_1_c.data || !img_2_c.data) {
//		std::cout << " --(!) Error reading images " << std::endl; return -1;
//	}
//
//	cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
//	cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);
//
//	// feature detection, tracking
//	vector<Point2f> points1, points2;        //vectors to store the coordinates of the feature points
//	featureDetection(img_1, points1);        //detect features in img_1
//	vector<uchar> status;
//	featureTracking(img_1, img_2, points1, points2, status); //track those features to img_2
//
//	double focal = 629.186;
//	cv::Point2d pp(319.5, 242.415);
//
//	// 첫번째, 두번째 카메라의 포즈 관계
//	Mat R_pos, t_pos;
//	Mat E, R, t, mask;
//	// E : 3 x 3
//	// R : 3 x 3
//	// t : 3 x 1
//	// mask는 매칭에 성공한 애들 저장
//	E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
//	recoverPose(E, points2, points1, R, t, focal, pp, mask);
//
//	Mat prevImage = img_2;
//	Mat currImage;
//	vector<Point2f> prevFeatures = points2;
//	vector<Point2f> currFeatures;
//
//	vector<Point3f> pnts3D;
//	Mat curr3DFeatures;			// 내가 추가한거
//
//	Mat K = Mat(3, 3, CV_64FC1, Scalar(0.));
//	Mat dist_coeffs_double;
//
//	K.at<double>(0, 0) = focal;
//	K.at<double>(0, 2) = 319.5;
//	K.at<double>(1, 1) = focal;
//	K.at<double>(1, 2) = 242.415;
//	K.at<double>(2, 2) = 1;
//
//	Mat K1 = Mat(3, 4, CV_64FC1, Scalar(0.));
//	K1.at<double>(0, 0) = 1.0;
//	K1.at<double>(1, 1) = 1.0;
//	K1.at<double>(2, 2) = 1.0;
//
//	Mat K2 = Mat(3, 4, CV_64FC1, Scalar(0.));
//	K2.at<double>(0, 0) = R.at<double>(0, 0);
//	K2.at<double>(1, 1) = R.at<double>(1, 1);
//	K2.at<double>(2, 2) = R.at<double>(2, 2);
//
//	K2.at<double>(0, 3) = t.at<double>(0, 0);
//	K2.at<double>(1, 3) = t.at<double>(1, 0);
//	K2.at<double>(2, 3) = t.at<double>(2, 0);
//
//	K1 = K * K1;
//	K2 = K * K2;
//
//	triangulatePoints(K1, K2, points1, points2, curr3DFeatures);
//
//	for (int x = 0; x < curr3DFeatures.cols; x++) {
//		float W = curr3DFeatures.at<float>(3, x);
//		float Z = curr3DFeatures.at<float>(2, x); /// 1000;
//
//		float X = curr3DFeatures.at<float>(0, x); /// 1000;
//		float Y = curr3DFeatures.at<float>(1, x); /// 1000;
//
//		pnts3D.push_back(Point3f(X, Y, Z));
//	}
//	
//	Mat depth = imread("./realsense_missing/depth_0001.png");
//	depth.convertTo(depth, CV_16SC1);
//	curr3DFeatures.convertTo(curr3DFeatures, CV_16SC4);
//
//	for (int j = 0; j < 480; j++) {
//		for (int m = 0; m < 640; m++) {
//			for (int n = 0; n < points2.size(); n++) {
//				if ((points2.at(n).y == j) && (points2.at(n).x == m)) {
//					cout << depth.at<ushort>(j, m) << " " << pnts3D.at(n).z << endl;
//				}
//			}
//		}
//	}
//	
//	char filename[100];
//
//	namedWindow("Road facing camera", WINDOW_AUTOSIZE);// Create a window for display.
//
//	for (int numFrame = 2; numFrame < MAX_FRAME; numFrame++) {
//		sprintf(filename, "./realsense_missing/img_%04d.png", numFrame);
//		Mat currImage_c = imread(filename);
//		cvtColor(currImage_c, currImage, COLOR_BGR2GRAY);
//		vector<uchar> status;
//		featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
//
//		// 매칭되는거 체크
//		E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.999, 1.0, mask);
//		recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
//
//		if (prevFeatures.size() < MIN_NUM_FEAT) {
//			featureDetection(prevImage, prevFeatures);				// 매칭 안되면 이전꺼 detection
//			featureTracking(prevImage, currImage, prevFeatures, currFeatures, status);
//
//			// cout << "hello world!\n";
//		}
//
//		for (Point2f pt : currFeatures) {
//			circle(currImage_c, pt, 5, Scalar(0, 0, 255), 2);
//		}
//
//		prevImage = currImage.clone();
//		prevFeatures = currFeatures;
//
//		imshow("Road facing camera", currImage_c);
//
//		waitKey(27);
//	}
//
//	return 0;
//}