#include <iostream>
#include <vector>
#include <io.h>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

using namespace std;
using namespace cv;

template<typename ... Args>
std::string string_format(const std::string& format, Args ... args) {
	size_t size = snprintf(nullptr, 0, format.c_str(), args ...) + 1;
	if (size <= 0) { throw std::runtime_error("Error during formatting."); }
	std::unique_ptr<char[]> buf(new char[size]); snprintf(buf.get(), size, format.c_str(), args ...);
	return std::string(buf.get(), buf.get() + size - 1);
}

void extract_features(
	vector<string>& image_names,
	vector<vector<KeyPoint>>& key_points_for_all,
	vector<Mat>& descriptor_for_all
);

void match_features(Mat& query, Mat& train, vector<DMatch>& matches);
void match_features(vector<Mat>& descriptor_for_all, vector<vector<DMatch>>& matches_for_all);

cv::Mat pull_push(cv::Mat input)
{
	int level = 5;
	unsigned short **pyr = new unsigned short*[level];

	int w = input.cols;
	int h = input.rows;
	int wtemp;
	int htemp;

	int idx, idx2, idx3, idx4, idx5, idx6;
	//Pyramid up 
	for (int i = 0; i < level; i++)
	{
		if (i == 0)
		{
			pyr[0] = new unsigned short[w*h];
			memcpy(pyr[0], input.data, sizeof(unsigned short)*w*h);
		}
		else
		{
			wtemp = w;
			w /= 2;
			h /= 2;
			pyr[i] = new unsigned short[w*h];
			memset(pyr[i], 0, sizeof(unsigned short)*w*h);
			for (int y = 0; y < h; y++)
			{
				for (int x = 0; x < w; x++)
				{
					int divider = 4; // variable to deal with invalid pixels
					idx = y * w + x;
					//Original Position
					idx2 = (y * 2 * wtemp + x * 2);
					//UP
					idx3 = idx2;
					//DOWN
					idx4 = idx2 + wtemp;
					//LEFT
					idx5 = idx2 + wtemp + 1;
					//RIGHT
					idx6 = idx2 + 1;
					if (pyr[i - 1][idx3] == 0)
						divider -= 1;
					if (pyr[i - 1][idx4] == 0)
						divider -= 1;
					if (pyr[i - 1][idx5] == 0)
						divider -= 1;
					if (pyr[i - 1][idx6] == 0)
						divider -= 1;
					if (divider == 0)
					{
						pyr[i][idx] = 0;
					}
					else
					{
						pyr[i][idx] = (pyr[i - 1][idx3] + pyr[i - 1][idx4] + pyr[i - 1][idx5] + pyr[i - 1][idx6]) / divider;
					}
				}
			}
		}
	}

	//Pyramid down
	for (int i = level - 2; i >= 0; i--)
	{
		wtemp = w;
		htemp = h;
		w *= 2;
		h *= 2;
		for (int y = 2; y < h - 2; y++) {
			for (int x = 2; x < w - 2; x++) {
				int divider = 5;
				int idx2 = y * w + x; // bigger pyr
				int idx = floor(y * 0.5) * wtemp + floor(x * 0.5); // smaller pyr
				if (pyr[i][y * w + x] == 0)
				{
					{ // while pixel around still invalid
					  //UP
						idx3 = idx - wtemp;
						//DOWN
						idx4 = idx + wtemp;
						//LEFT
						idx5 = idx - 1;
						//RIGHT
						idx6 = idx + 1;

						pyr[i][idx2] = (pyr[i + 1][idx] + pyr[i + 1][idx3] + pyr[i + 1][idx4] + pyr[i + 1][idx5] + pyr[i + 1][idx6]) / divider;
					}
				}
				if (pyr[i][y * w + x] < 400) pyr[i][y * w + x] = 400;
			}
		}
	}

	memcpy(input.data, pyr[0], sizeof(unsigned short)*w*h);

	return input;
}

#define num 10

int main() {

	vector<string> img_names;
	vector<string> depth_names;
	string filename;

	for (int i = 0; i < 200; i = i + num) {
		filename = string_format("M:/drive/Etri_Code/dataset/making_depth_data/original_test/img_%04d.png", i);
		img_names.push_back(filename);

		filename = string_format("M:/drive/Etri_Code/dataset/rgbd_dataset_freiburg1_xyz/naming_depth/depth_%04d.png", i);
		depth_names.push_back(filename);
	}

	vector<Mat> depth;
	Mat test;

	for (int i = 0; i < 200; i = i + num) {
		Mat pDepth = cv::imread(depth_names[i / num], -1);
		pull_push(pDepth);

		normalize(pDepth, test, 0, 255, NORM_MINMAX, CV_8U);
		imshow("test", test);
		waitKey(27);

		depth.push_back(pDepth);
	}
	

	// 내부인자
	Mat K(Matx33d(
		525.0, 0, 319.5,
		0, 525.0, 239.5,
		0, 0, 1
	));

	vector<vector<KeyPoint>> key_points_for_all;			// 이미지 당 키포인트
	vector<Mat> descriptor_for_all;							// 각 이미지 당 descriptor. 근데 왜 하나의 Matㅇ로 되어 있을까 궁금하다
	vector<vector<DMatch>> matches_for_all;					// all 이미지의 매치 정보

	// 각 이미지마다 keypoint, descriptor, color 구하기
	// input : img_names	,	output : key_points_for_all, descriptor_for_all
	extract_features(img_names, key_points_for_all, descriptor_for_all);

	// 이미지간의 descriptor를 통해 match 구하기?
	// input : descriptor_for_all	,	output : matches_for_all
	match_features(descriptor_for_all, matches_for_all);

	vector<Mat> rotations;						// 이미지 당 모든 rot
	vector<Mat> motions;						// 이미지 당 모든 trans?
	
	// R0, T0가 무슨 의미일까...?
	// 맨 처음 카메라의 기본 포즈!!!!
	// Mat R0 = Mat::eye(3, 3, CV_64FC1);
	Mat R0(Matx33d(
		0.07543147411083315, 0.613931888201209, -0.7857466063325789,
		0.9970987021421238, -0.038370247970284016, 0.06574117626868853,
		0.010211312352414292, -0.7884258752225202, -0.6150450132954858
	));

	rotations.push_back(R0);

	cout << "0" << "\n" << R0 << endl << endl;

	for (int i = 0; i < matches_for_all.size(); ++i)
	{
		vector<Point3f> object_points;
		vector<Point2f> image_points;
		Mat r, R, T;

		// i, i + 1과의 매칭 점들!!
		for (int j = 0; j < matches_for_all[i].size(); j++) {
			for (int k = 0; k < matches_for_all[i].size(); k++) {

				int query_idx = matches_for_all[i][k].queryIdx;
				int train_idx = matches_for_all[i][k].trainIdx;

				float temp = 0.f;

				for (int s1 = -1; s1 <= 1; s1++) {
					for (int s2 = -1; s2 <= 1; s2++) {
						temp += depth[i].at<float>(key_points_for_all[i][query_idx].pt.y + s2, key_points_for_all[i][query_idx].pt.x + s1);
					}
				}
				float z = temp / 9;
				// float z = depth[i].at<float>(key_points_for_all[i][query_idx].pt.y, key_points_for_all[i][query_idx].pt.x);
				float x = ((key_points_for_all[i][query_idx].pt.x - 320) / 525.0) * z;
				float y = ((key_points_for_all[i][query_idx].pt.y - 240) / 525.0) * z;

				object_points.push_back(Point3f(x, y, z));
				image_points.push_back(Point2f(key_points_for_all[i + 1][train_idx].pt.x, key_points_for_all[i + 1][train_idx].pt.y));

			}
		}

		// solve pnp를 통해 현재 r, t를 구한다
		solvePnPRansac(object_points, image_points, K, noArray(), r, T);

		// 제대로 된 R을 구한다
		Rodrigues(r, R);

		R0 = R * R0;

		// rotations, motions에 현재 정보를 추가!!
		rotations.push_back(R0);
		//motions.push_back(T);


		cout << (i + 1) * num << "\n" << R0 << endl << endl;
		//cout << T << endl << endl;
	}

	cout << "finish Rotation!!\n";

	return EXIT_SUCCESS;
}

void extract_features(
	vector<string>& image_names,
	vector<vector<KeyPoint>>& key_points_for_all,
	vector<Mat>& descriptor_for_all
)
{
	// 왜 두개를 초기화하고 시작하지 -> 맨 처음이니까. 나중에 코딩에서도 clear하고 시작하자
	key_points_for_all.clear();
	descriptor_for_all.clear();
	Mat image;

	Ptr<Feature2D> sift = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10);

	// 피처 매칭 후 sift란 곳에 피처 저장
	for (auto it = image_names.begin(); it != image_names.end(); ++it)
	{
		// 각 이미지들 생성
		image = imread(*it);			// 현재 이미지 불러오기
		if (image.empty())
		{
			continue;
		}
		cout << "Extracting features: " << *it << endl;

		vector<KeyPoint> key_points;
		Mat descriptor;
		// 현재 이미지에 대해 key_points와 descriptor 구하기
		// sift->detectAndCompute(image, noArray(), key_points, descriptor);
		sift->detect(image, key_points);
		sift->compute(image, key_points, descriptor);


		// key_point가 10개 이하면 없애기!
		if (key_points.size() <= 10)
		{
			continue;
		}

		// 이미지별로 모든 키포인트와 descriptor 구하기
		key_points_for_all.push_back(key_points);
		descriptor_for_all.push_back(descriptor);
	}
}

void match_features(vector<Mat>& descriptor_for_all, vector<vector<DMatch>>& matches_for_all)
{
	matches_for_all.clear();

	// 모든 디스크립터 계산 -> 하나 전이니까 처음부터 다음 사이즈까지!!!
	for (int i = 0; i < descriptor_for_all.size() - 1; ++i)
	{
		// 전후 이미지 두개를 모두 매칭한다
		cout << "Matching images " << i << " - " << i + 1 << endl;
		vector<DMatch> matches;

		// 매치들을 구해서 vector에 넣어줌 (아래 함수)
		match_features(descriptor_for_all[i], descriptor_for_all[i + 1], matches);
		// 매칭 정보 전부 다 저장. -> size는 하나 작겠네!!! -> 중간 중간에 하나씩
		matches_for_all.push_back(matches);
	}
}

// 전후 디스크립터 입력
void match_features(Mat& query, Mat& train, vector<DMatch>& matches)
{
	// 여러개를 매칭할건가? 왜 knn_matches을 vector로? -> 입력 두개가 0, 1에 저장된다!!!
	vector<vector<DMatch>> knn_matches;
	BFMatcher matcher(NORM_L2);

	// k = 2로 전후 이미지의 descriptor 매칭 진행
	matcher.knnMatch(query, train, knn_matches, 2);

	float min_dist = FLT_MAX;
	// 매치의 개수에 따라
	for (int r = 0; r < knn_matches.size(); ++r)
	{
		// Ratio Test
		// 두 이미지의 선택된 k의 사이가 너무 멀다면 그거는 그냥 넘긴다
		if (knn_matches[r][0].distance > 0.6 * knn_matches[r][1].distance)
		{
			continue;
		}

		// 가장 작은 k 거리를 저장한다 -> 왜...?
		float dist = knn_matches[r][0].distance;
		if (dist < min_dist)
		{
			min_dist = dist;
		}
	}

	matches.clear();
	for (size_t r = 0; r < knn_matches.size(); ++r)
	{
		// 너무 멀거나 최소 거리보다 5배 더 크다면 그냥 넘겨!! -> 너무 머니까 오류났을 확률이 너무 높거든!!
		if (
			knn_matches[r][0].distance > 0.6 * knn_matches[r][1].distance ||
			knn_matches[r][0].distance > 5 * max(min_dist, 10.0f)
			)
		{
			continue;
		}

		// 적당히 가깝다면 현재 search 이미지의 knn_matches 정보를 저장한다
		matches.push_back(knn_matches[r][0]);
	}
}