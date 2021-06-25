#include "basic.h"

void init_structure(
	Mat K,
	vector<vector<KeyPoint>>& key_points_for_all,
	vector<vector<Vec3b>>& colors_for_all,
	vector<vector<DMatch>>& matches_for_all,
	vector<Point3f>& structure,
	vector<vector<int>>& correspond_struct_idx,
	vector<Vec3b>& colors,
	vector<Mat>& rotations,
	vector<Mat>& motions
);
void get_matched_points(
	vector<KeyPoint>& p1,
	vector<KeyPoint>& p2,
	vector<DMatch> matches,
	vector<Point2f>& out_p1,
	vector<Point2f>& out_p2);
void get_matched_colors(
	vector<Vec3b>& c1,
	vector<Vec3b>& c2,
	vector<DMatch> matches,
	vector<Vec3b>& out_c1,
	vector<Vec3b>& out_c2
);
bool find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, Mat& R, Mat& T, Mat& mask);
void maskout_points(vector<Point2f>& p1, Mat& mask);
void maskout_colors(vector<Vec3b>& p1, Mat& mask);
void reconstruct(Mat& K, Mat& R1, Mat& T1, Mat& R2, Mat& T2, vector<Point2f>& p1, vector<Point2f>& p2, vector<Point3f>& structure);
void get_objpoints_and_imgpoints(
	vector<DMatch>& matches,
	vector<int>& struct_indices,
	vector<Point3f>& structure,
	vector<KeyPoint>& key_points,
	vector<Point3f>& object_points,
	vector<Point2f>& image_points);
void fusion_structure(
	vector<DMatch>& matches,
	vector<int>& struct_indices,
	vector<int>& next_struct_indices,
	vector<Point3f>& structure,
	vector<Point3f>& next_structure,
	vector<Vec3b>& colors,
	vector<Vec3b>& next_colors);
void extract_features(
	vector<string>& image_names,
	vector<vector<KeyPoint>>& key_points_for_all,
	vector<Mat>& descriptor_for_all,
	vector <vector<Vec3b>>& colors_for_all
);
void match_features(Mat& query, Mat& train, vector<DMatch>& matches);
void match_features(vector<Mat>& descriptor_for_all, vector<vector<DMatch>>& matches_for_all);
void save_structure(string file_name, vector<Mat>& rotations, vector<Mat>& motions, vector<Point3f>& structure, vector<Vec3b>& colors);

vector<string> img_names;
vector<string> depth_names;

void cameraInit()
{
	string filename;

	for (int i = 0; i < 500; i = i + num) {
		filename = dir + string_format("img_%04d.png", i);
		img_names.push_back(filename);

		Mat test = imread(filename, 0);
		imshow("test", test);
		waitKey(27);

		filename = "M:/drive/Etri_Code/mypython/costvolume_based_network(IWAIT2021 version)/original_depth_2/" + string_format("depth_%04d.png", i);
		depth_names.push_back(filename);
	}

	vector<vector<KeyPoint>> key_points_for_all;			// �̹��� �� Ű����Ʈ
	vector<Mat> descriptor_for_all;							// �� �̹��� �� descriptor. �ٵ� �� �ϳ��� Mat���� �Ǿ� ������ �ñ��ϴ�
	vector<vector<Vec3b>> colors_for_all;					// all �̹����� color
	vector<vector<DMatch>> matches_for_all;					// all �̹����� ��ġ ����

	// �� �̹������� keypoint, descriptor, color ���ϱ�
	// input : img_names	,	output : key_points_for_all, descriptor_for_all, colors_for_all
	extract_features(img_names, key_points_for_all, descriptor_for_all, colors_for_all);

	// �̹������� descriptor�� ���� match ���ϱ�?
	// input : descriptor_for_all	,	output : matches_for_all
	match_features(descriptor_for_all, matches_for_all);

	// structure�� ����...
	//vector<Point3f> structure;					// ��ü point�� ��ġ �����ΰ�??
	vector<vector<int>> correspond_struct_idx;	// �� idx�� �������� �˾ƺ���
	vector<Vec3b> colors;						// ���� �̹����ΰ�??
	//vector<Mat> rotations;						// �̹��� �� ��� rot
	//vector<Mat> motions;						// �̹��� �� ��� trans?

	// structure�� ����µ� ���⼭ rotations, motions�� ���ϴ°ǰ�?
	// input : K, key_points_for_all, colors_for_all, matches_for_all
	// output : structure, correspond_struct_idx, colors, rotations, motions
	init_structure(
		K,
		key_points_for_all,
		colors_for_all,
		matches_for_all,
		structure,
		correspond_struct_idx,
		colors,
		rotations,
		motions
	);

	// SFM�� ���� 3D object ����ȭ? -> ��� ��Ī ����Ʈ???
	// ó�� �ΰ��� match�� ���� world point���� ��� ���� �͵��� ���� �� �ϳ�...?
	// �� ��ġ�� ���� ��� ���� �����̾�
	for (int i = 1; i < matches_for_all.size(); ++i)
	{
		// 3D, 2D�� ���� point���̳�. �̰� ���߿� solvepnp �� ����
		vector<Point3f> object_points;
		vector<Point2f> image_points;
		Mat r, R, T;

		// input : matches_for_all[i], correspond_struct_idx[i],
		// input : normalize �� ���������� structure, ���� ��� keypoint : key_points_for_all[i + 1]
		// output : object_points, image_points ( world, image point )
		// structure idx�� �޾Ƽ� ����ϴ±��� -> ���� structure, match�� ���� ���� image, obj ���� ���Ѵ�
		get_objpoints_and_imgpoints(
			matches_for_all[i],
			correspond_struct_idx[i],
			structure,
			key_points_for_all[i + 1],
			object_points,
			image_points
		);

		// solve pnp�� ���� ���� r, t�� ���Ѵ�
		solvePnPRansac(object_points, image_points, K, noArray(), r, T);

		// ����� �� R�� ���Ѵ�
		Rodrigues(r, R);

		// rotations, motions�� ���� ������ �߰�!!
		rotations.push_back(R);
		motions.push_back(T);

		// key_points_for_all[i], key_points_for_all[i + 1] ���� ��Ī�Ǵ� �� ��, color ������ ��� �����Ѵ�
		vector<Point2f> p1, p2;
		vector<Vec3b> c1, c2;
		get_matched_points(key_points_for_all[i], key_points_for_all[i + 1], matches_for_all[i], p1, p2);
		get_matched_colors(colors_for_all[i], colors_for_all[i + 1], matches_for_all[i], c1, c2);

		// ���� ���ϴ� �ΰ��� ������ structure�� �ٽ� �����
		vector<Point3f> next_structure;
		reconstruct(K, rotations[i], motions[i], R, T, p1, p2, next_structure);

		// structure, next_structure �ΰ��� ���� �� ����°���??
		// input : matches_for_all[i] ( ���ϴ� �ΰ��� ��Ī ���� )
		// input : correspond_struct_idx[i]		,	output : correspond_struct_idx[i + 1] -> ���ο� stucture�� index�� ���ϴ°ǰ�
		fusion_structure(
			matches_for_all[i],
			correspond_struct_idx[i],
			correspond_struct_idx[i + 1],
			structure,
			next_structure,
			colors,
			c1
		);

		// �ᱹ structure�� ��û �þ�ڴ�
	}

	//save_structure("structure.yml", rotations, motions, structure, colors);
	cout << "successful!!!" << endl;
	//getchar();
	//return 0;
}

void save_structure(string file_name, vector<Mat>& rotations, vector<Mat>& motions, vector<Point3f>& structure, vector<Vec3b>& colors)
{
	int n = (int)rotations.size();

	FileStorage fs(file_name, FileStorage::WRITE);
	fs << "Camera Count" << n;
	fs << "Point Count" << (int)structure.size();

	fs << "Rotations" << "[";
	for (size_t i = 0; i < n; ++i)
	{
		fs << rotations[i];
	}
	fs << "]";

	cout << rotations[0].type() << endl;	// ����

	fs << "Motions" << "[";
	for (size_t i = 0; i < n; i++)
	{
		fs << motions[i];
	}
	fs << "]";

	cout << motions[0].type() << endl;	// ����

	fs << "Points" << "[";
	for (size_t i = 0; i < structure.size(); ++i)
	{
		fs << structure[i];
	}
	fs << "]";

	fs << "Colors" << "[";
	for (size_t i = 0; i < colors.size(); ++i)
	{
		fs << colors[i];
	}
	fs << "]";

	fs.release();

}

void extract_features(
	vector<string>& image_names,
	vector<vector<KeyPoint>>& key_points_for_all,
	vector<Mat>& descriptor_for_all,
	vector <vector<Vec3b>>& colors_for_all
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

		// keypoint�� �ش��ϴ� ��� color�� �����ϱ�
		vector<Vec3b> colors(key_points.size());
		for (int i = 0; i < key_points.size(); ++i)
		{
			Point2f& p = key_points[i].pt;
			if (p.x <= image.rows && p.y <= image.cols)
				colors[i] = image.at<Vec3b>(p.x, p.y);		// �̷��Ե� ����� �� �ֱ���
		}

		colors_for_all.push_back(colors);					// ��� �̹����� ������ �� �����ϱ�
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


void init_structure(
	Mat K,
	vector<vector<KeyPoint>>& key_points_for_all,
	vector<vector<Vec3b>>& colors_for_all,
	vector<vector<DMatch>>& matches_for_all,
	vector<Point3f>& structure,
	vector<vector<int>>& correspond_struct_idx,
	vector<Vec3b>& colors,
	vector<Mat>& rotations,
	vector<Mat>& motions
)
{
	// ��ġ�� point �����ϱ� -> 2D!!!
	vector<Point2f> p1, p2;
	// ��ġ�� color�� �����ϱ�
	vector<Vec3b> c2;

	Mat R, T;	// ī�޶� R, T
	Mat mask;	// mask�� �� ���� -> find transform���� ���ϳ�?

	// �� ó�� �ΰ� �̹����� keypoint�� match�� �ش��ϴ� point�� p1, p2�� �����Ѵ�
	get_matched_points(key_points_for_all[0], key_points_for_all[1], matches_for_all[0], p1, p2);
	get_matched_colors(colors_for_all[0], colors_for_all[1], matches_for_all[0], colors, c2);

	// input : ī�޶� ���� ����, �̹����� ��Ī�Ǵ� points
	// output : R, T, mask
	// mask�� 0�� ���� ��ҵ��� ������ �־� �̰� ���߿� ��� ���̷���...?
	find_transform(K, p1, p2, R, T, mask);	// ���Ƿֽ�õ�R�� T ����

	// mask.rows�� ������ŭ point. color�� �ٽ� �����Ѵ�
	maskout_points(p1, mask);
	maskout_points(p2, mask);
	maskout_colors(colors, mask);

	// R0, T0�� ���� �ǹ��ϱ�...?
	// �� ó�� ī�޶��� �⺻ ����!!!!
	Mat R0 = Mat::eye(3, 3, CV_64FC1);
	Mat T0 = Mat::zeros(3, 1, CV_64FC1);

	// input : K, R, T, p1, p2
	// output : R0, T0, structure (3D obj point)
	reconstruct(K, R0, T0, R, T, p1, p2, structure);	// ���� ī�޶� ���� s point�� ���ϴ°ǰ�? ��ư homo �ܿ� ��� ����Ʈ ��ġ ���Ѵ�!!

	// ó�� �ΰ��� R, T�� �̸� �����Ѵ�
	rotations = { R0, R };
	motions = { T0, T };

	// ��correspond_struct_idx�Ĵ�С��ʼ��Ϊ��key_points_for_all��ȫһ��

	// vector<vector<int>>& correspond_struct_idx
	// correspond_struct_idx�� �ǹ̸� ���� �� �𸣰ھ� -> �ϴ� ����� �ʱ�ȭ �����̾�
	correspond_struct_idx.clear();
	// key_points_for_all �� ���� size�� �����ϰ�!!
	// �̹������� ��� Ű����Ʈ. �� �̹��� ����� �޴´�
	correspond_struct_idx.resize(key_points_for_all.size());
	for (int i = 0; i < key_points_for_all.size(); ++i)
	{
		// correspond_struct_idx[i] �� �̹��� �� ����� -1�� �ʱ�ȭ�Ѵ�!!!
		correspond_struct_idx[i].resize(key_points_for_all[i].size(), -1);
	}

	// �ϴ� �� ó�� ��ġ�� Ȯ��
	int idx = 0;
	vector<DMatch>& matches = matches_for_all[0];

	// ��ȿ�� matches�� Ȯ������!!
	for (int i = 0; i < matches.size(); ++i)
	{
		if (mask.at<uchar>(i) == 0)
		{
			continue;
		}

		// ��ȿ�� match�� �ش��ϴ� point�鿡 idx�� �����Ѵ�
		// queryIdx, trainIdx�� ������ ���ؼ� point�� ������ �����̾�
		// ����� �� point���� idx�� ǥ���Ѵ�!!!
		correspond_struct_idx[0][matches[i].queryIdx] = idx;
		correspond_struct_idx[1][matches[i].trainIdx] = idx;
		++idx;
	}
}

void get_matched_points(
	vector<KeyPoint>& p1,
	vector<KeyPoint>& p2,
	vector<DMatch> matches,
	vector<Point2f>& out_p1,
	vector<Point2f>& out_p2)
{
	out_p1.clear();
	out_p2.clear();
	for (int i = 0; i < matches.size(); ++i)
	{
		out_p1.push_back(p1[matches[i].queryIdx].pt);
		out_p2.push_back(p2[matches[i].trainIdx].pt);
	}
}

void get_matched_colors(
	vector<Vec3b>& c1,
	vector<Vec3b>& c2,
	vector<DMatch> matches,
	vector<Vec3b>& out_c1,
	vector<Vec3b>& out_c2
)
{
	out_c1.clear();
	out_c2.clear();
	for (int i = 0; i < matches.size(); ++i)
	{
		out_c1.push_back(c1[matches[i].queryIdx]);
		out_c2.push_back(c2[matches[i].trainIdx]);
	}
}

bool find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, Mat& R, Mat& T, Mat& mask)
{
	// fx, fy�� ��հ��� ���ؼ� ����
	double focal_length = 0.5 * (K.at<double>(0) + K.at<double>(4));

	// shearing �� ����?
	Point2d principle_point(K.at<double>(2), K.at<double>(5));

	// point �ΰ��� �� ��Ī�ؼ� E�� ���Ѵ�. mask�� ����
	Mat E = findEssentialMat(p1, p2, focal_length, principle_point, RANSAC, 0.999, 1.0, mask);
	if (E.empty())
	{
		return false;
	}

	double feasible_count = countNonZero(mask);	// 0�� �ƴ� ��� �Ǻ� -> ���� ����?
	// cout << (int)feasible_count << " - in - " << p1.size() << endl;

	// RANSAC�� ����� 15 ����, point�� ���� ������ �ȵǸ� ���� ���!!
	if (feasible_count <= 15 || (feasible_count / p1.size()) < 0.6)
	{
		return false;
	}

	// E, mask, p1, p2�� ������ R, T�� ����� ���Ѵ�!! -> count�� point �� ����
	int pass_count = recoverPose(E, p1, p2, R, T, focal_length, principle_point, mask);

	// cout << "pass_count = " << pass_count << endl;

	// ī��Ʈ�� ������ ���� ���� ���϶�� ���� ���!!
	if (((double)pass_count) / feasible_count < 0.7)
	{
		return false;
	}
	return true;
}

void maskout_points(vector<Point2f>& p1, Mat& mask)
{
	vector<Point2f> p1_copy = p1;
	p1.clear();

	for (int i = 0; i < mask.rows; ++i)
	{
		if (mask.at<uchar>(i) > 0)
		{
			p1.push_back(p1_copy[i]);
		}
	}
}

void maskout_colors(vector<Vec3b>& p1, Mat& mask)
{
	vector<Vec3b> p1_copy = p1;
	p1.clear();

	for (int i = 0; i < mask.rows; ++i)
	{
		if (mask.at<uchar>(i) > 0)
		{
			p1.push_back(p1_copy[i]);
		}
	}
}

void reconstruct(Mat& K, Mat& R1, Mat& T1, Mat& R2, Mat& T2, vector<Point2f>& p1, vector<Point2f>& p2, vector<Point3f>& structure)
{
	// ���۽���...? �� �����ϴ°ž� -> ���� ī�޶��� world ��ǥ ���ϴ� ��̰� �Ƿ��±���!!
	Mat proj1(3, 4, CV_32FC1);
	Mat proj2(3, 4, CV_32FC1);

	// R1, T1�� ���⿡ �ִ´�! -> ó�����ϱ� R0, T0�� ���±���~
	R1.convertTo(proj1(Range(0, 3), Range(0, 3)), CV_32FC1);
	//T1.convertTo(proj2(Range(0, 3), Range(3, 4)), CV_32FC1);
	T1.convertTo(proj1.col(3), CV_32FC1);

	// R2, T2�� ���⿡ �ִ´�! -> ����� ���� ���� ������
	R2.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
	//T2.convertTo(proj2(Range(0, 3), Range(3, 4)), CV_32FC1);
	T2.convertTo(proj2.col(3), CV_32FC1);

	// �ΰ� ī�޶��� point�� homogenius ���� ���Ѵ�
	Mat fK;
	K.convertTo(fK, CV_32FC1);
	proj1 = fK * proj1;
	proj2 = fK * proj2;

	// ���۽����� s...? s�� �������� -> ���⼭ ���� z�� ���� ���� �� ���� �� ����!!
	// s�� point 4D��. ���� s�� ���ϴ� ���ϱ� ( points4D )

	Mat s;
	triangulatePoints(proj1, proj2, p1, p2, s);

	structure.clear();
	structure.reserve(s.cols);			// s�� ������ŭ �̸� ������ Ȯ������!
	for (int i = 0; i < s.cols; ++i)
	{
		Mat_<float> col = s.col(i);		// 1, 2, 3, 4 �ִµ� h�� homo �� �κ��ΰ���
		// homogenius ������ �����༭ ���⼭ ������ point���� ���ϴ°�...?
		col /= col(3);					// normalize
		// normalize �� ���� �����Ѵ�!! -> �׷� ��Ī�� real data ��ġ ���Ѵ� (ó�� ����?)
		structure.push_back(Point3f(col(0), col(1), col(2)));
	}
}

void get_objpoints_and_imgpoints(
	vector<DMatch>& matches,
	vector<int>& struct_indices,
	vector<Point3f>& structure,
	vector<KeyPoint>& key_points,
	vector<Point3f>& object_points,
	vector<Point2f>& image_points)
{
	object_points.clear();
	image_points.clear();

	// ��� ��ġ point�� ���Ѵ�
	for (int i = 0; i < matches.size(); ++i)
	{
		// keypoint idx�� �޴´�
		int query_idx = matches[i].queryIdx;
		int train_idx = matches[i].trainIdx;

		int struct_idx = struct_indices[query_idx];
		if (struct_idx < 0)				// -1�� ���� ���ٸ� �׳� �������� -> �̰� ���� �� ȿ�������� �� �� ���� ��?
		{
			continue;
		}

		object_points.push_back(structure[struct_idx]);		// ���� point�鿡 ���� ��Ī�Ǵ� world point�� �����Ѵ�
		image_points.push_back(key_points[train_idx].pt);	// ���� point�鿡 ���� ��Ī�Ǵ� keyoint�� �����Ѵ�
	}
}

void fusion_structure(
	vector<DMatch>& matches,
	vector<int>& struct_indices,
	vector<int>& next_struct_indices,
	vector<Point3f>& structure,
	vector<Point3f>& next_structure,
	vector<Vec3b>& colors,
	vector<Vec3b>& next_colors)
{

	// ���� ���ϴ� ��ġ�� ��� ����!! -> �׷��� vector<DMatch> �̰ž�
	for (int i = 0; i < matches.size(); ++i)
	{
		int query_idx = matches[i].queryIdx;
		int train_idx = matches[i].trainIdx;

		// struct_indices : ���� sturcture�� idx ����
		int struct_idx = struct_indices[query_idx];
		if (struct_idx >= 0)	// struct_indices�� ���� ��ȿ�ϴٸ� struct_idx�� ��Ī�Ǵ� ���� ǥ��!!
		{
			next_struct_indices[train_idx] = struct_idx;
			continue;
		}

		// ��Ī�� �ȵȴٸ� �ش� ���� ���ο� ������ �߰��Ѵ�
		// ���� structure ���⿡ next_structure�� i ��° ģ���� �߰� �����Ѵ�
		structure.push_back(next_structure[i]);
		colors.push_back(next_colors[i]);

		// structure.size() - 1 ( index ���� ���� )
		struct_indices[query_idx] = next_struct_indices[train_idx] = structure.size() - 1;
	}
}