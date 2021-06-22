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
	

	// ��������
	Mat K(Matx33d(
		525.0, 0, 319.5,
		0, 525.0, 239.5,
		0, 0, 1
	));

	vector<vector<KeyPoint>> key_points_for_all;			// �̹��� �� Ű����Ʈ
	vector<Mat> descriptor_for_all;							// �� �̹��� �� descriptor. �ٵ� �� �ϳ��� Mat���� �Ǿ� ������ �ñ��ϴ�
	vector<vector<DMatch>> matches_for_all;					// all �̹����� ��ġ ����

	// �� �̹������� keypoint, descriptor, color ���ϱ�
	// input : img_names	,	output : key_points_for_all, descriptor_for_all
	extract_features(img_names, key_points_for_all, descriptor_for_all);

	// �̹������� descriptor�� ���� match ���ϱ�?
	// input : descriptor_for_all	,	output : matches_for_all
	match_features(descriptor_for_all, matches_for_all);

	vector<Mat> rotations;						// �̹��� �� ��� rot
	vector<Mat> motions;						// �̹��� �� ��� trans?
	
	// R0, T0�� ���� �ǹ��ϱ�...?
	// �� ó�� ī�޶��� �⺻ ����!!!!
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

		// i, i + 1���� ��Ī ����!!
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

		// solve pnp�� ���� ���� r, t�� ���Ѵ�
		solvePnPRansac(object_points, image_points, K, noArray(), r, T);

		// ����� �� R�� ���Ѵ�
		Rodrigues(r, R);

		R0 = R * R0;

		// rotations, motions�� ���� ������ �߰�!!
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
	// �� �ΰ��� �ʱ�ȭ�ϰ� �������� -> �� ó���̴ϱ�. ���߿� �ڵ������� clear�ϰ� ��������
	key_points_for_all.clear();
	descriptor_for_all.clear();
	Mat image;

	Ptr<Feature2D> sift = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10);

	// ��ó ��Ī �� sift�� ���� ��ó ����
	for (auto it = image_names.begin(); it != image_names.end(); ++it)
	{
		// �� �̹����� ����
		image = imread(*it);			// ���� �̹��� �ҷ�����
		if (image.empty())
		{
			continue;
		}
		cout << "Extracting features: " << *it << endl;

		vector<KeyPoint> key_points;
		Mat descriptor;
		// ���� �̹����� ���� key_points�� descriptor ���ϱ�
		// sift->detectAndCompute(image, noArray(), key_points, descriptor);
		sift->detect(image, key_points);
		sift->compute(image, key_points, descriptor);


		// key_point�� 10�� ���ϸ� ���ֱ�!
		if (key_points.size() <= 10)
		{
			continue;
		}

		// �̹������� ��� Ű����Ʈ�� descriptor ���ϱ�
		key_points_for_all.push_back(key_points);
		descriptor_for_all.push_back(descriptor);
	}
}

void match_features(vector<Mat>& descriptor_for_all, vector<vector<DMatch>>& matches_for_all)
{
	matches_for_all.clear();

	// ��� ��ũ���� ��� -> �ϳ� ���̴ϱ� ó������ ���� ���������!!!
	for (int i = 0; i < descriptor_for_all.size() - 1; ++i)
	{
		// ���� �̹��� �ΰ��� ��� ��Ī�Ѵ�
		cout << "Matching images " << i << " - " << i + 1 << endl;
		vector<DMatch> matches;

		// ��ġ���� ���ؼ� vector�� �־��� (�Ʒ� �Լ�)
		match_features(descriptor_for_all[i], descriptor_for_all[i + 1], matches);
		// ��Ī ���� ���� �� ����. -> size�� �ϳ� �۰ڳ�!!! -> �߰� �߰��� �ϳ���
		matches_for_all.push_back(matches);
	}
}

// ���� ��ũ���� �Է�
void match_features(Mat& query, Mat& train, vector<DMatch>& matches)
{
	// �������� ��Ī�Ұǰ�? �� knn_matches�� vector��? -> �Է� �ΰ��� 0, 1�� ����ȴ�!!!
	vector<vector<DMatch>> knn_matches;
	BFMatcher matcher(NORM_L2);

	// k = 2�� ���� �̹����� descriptor ��Ī ����
	matcher.knnMatch(query, train, knn_matches, 2);

	float min_dist = FLT_MAX;
	// ��ġ�� ������ ����
	for (int r = 0; r < knn_matches.size(); ++r)
	{
		// Ratio Test
		// �� �̹����� ���õ� k�� ���̰� �ʹ� �ִٸ� �װŴ� �׳� �ѱ��
		if (knn_matches[r][0].distance > 0.6 * knn_matches[r][1].distance)
		{
			continue;
		}

		// ���� ���� k �Ÿ��� �����Ѵ� -> ��...?
		float dist = knn_matches[r][0].distance;
		if (dist < min_dist)
		{
			min_dist = dist;
		}
	}

	matches.clear();
	for (size_t r = 0; r < knn_matches.size(); ++r)
	{
		// �ʹ� �ְų� �ּ� �Ÿ����� 5�� �� ũ�ٸ� �׳� �Ѱ�!! -> �ʹ� �Ӵϱ� �������� Ȯ���� �ʹ� ���ŵ�!!
		if (
			knn_matches[r][0].distance > 0.6 * knn_matches[r][1].distance ||
			knn_matches[r][0].distance > 5 * max(min_dist, 10.0f)
			)
		{
			continue;
		}

		// ������ �����ٸ� ���� search �̹����� knn_matches ������ �����Ѵ�
		matches.push_back(knn_matches[r][0]);
	}
}