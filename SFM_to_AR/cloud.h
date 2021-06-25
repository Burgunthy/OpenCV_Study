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

	vector<vector<KeyPoint>> key_points_for_all;			// 이미지 당 키포인트
	vector<Mat> descriptor_for_all;							// 각 이미지 당 descriptor. 근데 왜 하나의 Mat으로 되어 있을까 궁금하다
	vector<vector<Vec3b>> colors_for_all;					// all 이미지의 color
	vector<vector<DMatch>> matches_for_all;					// all 이미지의 매치 정보

	// 각 이미지마다 keypoint, descriptor, color 구하기
	// input : img_names	,	output : key_points_for_all, descriptor_for_all, colors_for_all
	extract_features(img_names, key_points_for_all, descriptor_for_all, colors_for_all);

	// 이미지간의 descriptor를 통해 match 구하기?
	// input : descriptor_for_all	,	output : matches_for_all
	match_features(descriptor_for_all, matches_for_all);

	// structure가 뭘까...
	//vector<Point3f> structure;					// 전체 point의 위치 정보인가??
	vector<vector<int>> correspond_struct_idx;	// 이 idx가 무엇인지 알아보기
	vector<Vec3b> colors;						// 현재 이미지인가??
	//vector<Mat> rotations;						// 이미지 당 모든 rot
	//vector<Mat> motions;						// 이미지 당 모든 trans?

	// structure를 만드는데 여기서 rotations, motions을 구하는건가?
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

	// SFM을 통한 3D object 안정화? -> 모든 매칭 포인트???
	// 처음 두개의 match에 대한 world point들을 얻고 다음 것들을 통해 뭘 하네...?
	// 각 매치에 따라 계속 비교할 예정이야
	for (int i = 1; i < matches_for_all.size(); ++i)
	{
		// 3D, 2D에 대한 point들이네. 이걸 나중에 solvepnp 할 예정
		vector<Point3f> object_points;
		vector<Point2f> image_points;
		Mat r, R, T;

		// input : matches_for_all[i], correspond_struct_idx[i],
		// input : normalize 된 이전까지의 structure, 다음 모든 keypoint : key_points_for_all[i + 1]
		// output : object_points, image_points ( world, image point )
		// structure idx를 받아서 계산하는구나 -> 이전 structure, match에 따라 현재 image, obj 값을 구한다
		get_objpoints_and_imgpoints(
			matches_for_all[i],
			correspond_struct_idx[i],
			structure,
			key_points_for_all[i + 1],
			object_points,
			image_points
		);

		// solve pnp를 통해 현재 r, t를 구한다
		solvePnPRansac(object_points, image_points, K, noArray(), r, T);

		// 제대로 된 R을 구한다
		Rodrigues(r, R);

		// rotations, motions에 현재 정보를 추가!!
		rotations.push_back(R);
		motions.push_back(T);

		// key_points_for_all[i], key_points_for_all[i + 1] 에서 매칭되는 두 점, color 정보를 모두 저장한다
		vector<Point2f> p1, p2;
		vector<Vec3b> c1, c2;
		get_matched_points(key_points_for_all[i], key_points_for_all[i + 1], matches_for_all[i], p1, p2);
		get_matched_colors(colors_for_all[i], colors_for_all[i + 1], matches_for_all[i], c1, c2);

		// 현재 비교하는 두개를 가지고 structure를 다시 만든다
		vector<Point3f> next_structure;
		reconstruct(K, rotations[i], motions[i], R, T, p1, p2, next_structure);

		// structure, next_structure 두개를 통해 뭘 만드는거지??
		// input : matches_for_all[i] ( 비교하는 두개의 매칭 정보 )
		// input : correspond_struct_idx[i]		,	output : correspond_struct_idx[i + 1] -> 새로운 stucture의 index를 구하는건가
		fusion_structure(
			matches_for_all[i],
			correspond_struct_idx[i],
			correspond_struct_idx[i + 1],
			structure,
			next_structure,
			colors,
			c1
		);

		// 결국 structure가 엄청 늘어나겠다
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

	cout << rotations[0].type() << endl;	// 여기

	fs << "Motions" << "[";
	for (size_t i = 0; i < n; i++)
	{
		fs << motions[i];
	}
	fs << "]";

	cout << motions[0].type() << endl;	// 여기

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

		// keypoint에 해당하는 모든 color들 저장하기
		vector<Vec3b> colors(key_points.size());
		for (int i = 0; i < key_points.size(); ++i)
		{
			Point2f& p = key_points[i].pt;
			if (p.x <= image.rows && p.y <= image.cols)
				colors[i] = image.at<Vec3b>(p.x, p.y);		// 이렇게도 사용할 수 있구나
		}

		colors_for_all.push_back(colors);					// 모든 이미지의 정보들 다 저장하기
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
	// 매치된 point 저장하기 -> 2D!!!
	vector<Point2f> p1, p2;
	// 매치된 color들 저장하기
	vector<Vec3b> c2;

	Mat R, T;	// 카메라 R, T
	Mat mask;	// mask는 또 뭐야 -> find transform에서 구하네?

	// 맨 처음 두개 이미지와 keypoint와 match에 해당하는 point를 p1, p2에 저장한다
	get_matched_points(key_points_for_all[0], key_points_for_all[1], matches_for_all[0], p1, p2);
	get_matched_colors(colors_for_all[0], colors_for_all[1], matches_for_all[0], colors, c2);

	// input : 카메라 내부 인자, 이미지의 매칭되는 points
	// output : R, T, mask
	// mask는 0이 없는 요소들을 가지고 있어 이게 나중에 어떻게 쓰이려나...?
	find_transform(K, p1, p2, R, T, mask);	// 힛실롸썩돤돕R， T 앤黎

	// mask.rows의 개수만큼 point. color를 다시 지정한다
	maskout_points(p1, mask);
	maskout_points(p2, mask);
	maskout_colors(colors, mask);

	// R0, T0가 무슨 의미일까...?
	// 맨 처음 카메라의 기본 포즈!!!!
	Mat R0 = Mat::eye(3, 3, CV_64FC1);
	Mat T0 = Mat::zeros(3, 1, CV_64FC1);

	// input : K, R, T, p1, p2
	// output : R0, T0, structure (3D obj point)
	reconstruct(K, R0, T0, R, T, p1, p2, structure);	// 현재 카메라 기준 s point를 구하는건가? 여튼 homo 외에 모든 포인트 위치 구한다!!

	// 처음 두개의 R, T를 미리 저장한다
	rotations = { R0, R };
	motions = { T0, T };

	// 쉥correspond_struct_idx돨댕鬼놓迦뺏槨宅key_points_for_all供홍寧鈴

	// vector<vector<int>>& correspond_struct_idx
	// correspond_struct_idx의 의미를 아직 잘 모르겠어 -> 일단 여기는 초기화 과정이야
	correspond_struct_idx.clear();
	// key_points_for_all 의 값과 size를 동일하게!!
	// 이미지별로 모든 키포인트. 즉 이미지 사이즈를 받는다
	correspond_struct_idx.resize(key_points_for_all.size());
	for (int i = 0; i < key_points_for_all.size(); ++i)
	{
		// correspond_struct_idx[i] 의 이미지 내 사이즈를 -1로 초기화한다!!!
		correspond_struct_idx[i].resize(key_points_for_all[i].size(), -1);
	}

	// 일단 맨 처음 매치를 확인
	int idx = 0;
	vector<DMatch>& matches = matches_for_all[0];

	// 유효한 matches만 확인하자!!
	for (int i = 0; i < matches.size(); ++i)
	{
		if (mask.at<uchar>(i) == 0)
		{
			continue;
		}

		// 유효한 match에 해당하는 point들에 idx에 저장한다
		// queryIdx, trainIdx의 정보를 통해서 point에 접근할 예정이야
		// 여기는 각 point들의 idx를 표현한다!!!
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
	// fx, fy의 평균값을 구해서 저장
	double focal_length = 0.5 * (K.at<double>(0) + K.at<double>(4));

	// shearing 값 저장?
	Point2d principle_point(K.at<double>(2), K.at<double>(5));

	// point 두개를 다 매칭해서 E를 구한다. mask는 뭘까
	Mat E = findEssentialMat(p1, p2, focal_length, principle_point, RANSAC, 0.999, 1.0, mask);
	if (E.empty())
	{
		return false;
	}

	double feasible_count = countNonZero(mask);	// 0이 아닌 요소 판별 -> 개수 저장?
	// cout << (int)feasible_count << " - in - " << p1.size() << endl;

	// RANSAC의 결과가 15 이하, point의 일정 비율이 안되면 오류 출력!!
	if (feasible_count <= 15 || (feasible_count / p1.size()) < 0.6)
	{
		return false;
	}

	// E, mask, p1, p2를 가지고 R, T를 제대로 구한다!! -> count된 point 값 저장
	int pass_count = recoverPose(E, p1, p2, R, T, focal_length, principle_point, mask);

	// cout << "pass_count = " << pass_count << endl;

	// 카운트된 개수가 일정 수준 이하라면 오류 출력!!
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
	// 갑작스레...? 왜 선언하는거야 -> 현재 카메라의 world 좌표 구하는 어레이가 되려는구나!!
	Mat proj1(3, 4, CV_32FC1);
	Mat proj2(3, 4, CV_32FC1);

	// R1, T1을 여기에 넣는다! -> 처음꺼니까 R0, T0가 들어가는구나~
	R1.convertTo(proj1(Range(0, 3), Range(0, 3)), CV_32FC1);
	//T1.convertTo(proj2(Range(0, 3), Range(3, 4)), CV_32FC1);
	T1.convertTo(proj1.col(3), CV_32FC1);

	// R2, T2을 여기에 넣는다! -> 여기는 구한 값이 들어가야지
	R2.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
	//T2.convertTo(proj2(Range(0, 3), Range(3, 4)), CV_32FC1);
	T2.convertTo(proj2.col(3), CV_32FC1);

	// 두개 카메라의 point의 homogenius 값을 구한다
	Mat fK;
	K.convertTo(fK, CV_32FC1);
	proj1 = fK * proj1;
	proj2 = fK * proj2;

	// 갑작스러운 s...? s는 누구세요 -> 여기서 뭔가 z를 통해 구할 수 있을 것 같아!!
	// s는 point 4D야. 과연 s는 뭐하는 놈일까 ( points4D )

	Mat s;
	triangulatePoints(proj1, proj2, p1, p2, s);

	structure.clear();
	structure.reserve(s.cols);			// s의 개수만큼 미리 공간을 확보하자!
	for (int i = 0; i < s.cols; ++i)
	{
		Mat_<float> col = s.col(i);		// 1, 2, 3, 4 있는데 h가 homo 그 부분인거지
		// homogenius 값으로 나눠줘서 여기서 임의의 point들을 구하는거...?
		col /= col(3);					// normalize
		// normalize 된 것을 저장한다!! -> 그럼 매칭된 real data 위치 구한다 (처음 기준?)
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

	// 모든 매치 point를 비교한다
	for (int i = 0; i < matches.size(); ++i)
	{
		// keypoint idx를 받는다
		int query_idx = matches[i].queryIdx;
		int train_idx = matches[i].trainIdx;

		int struct_idx = struct_indices[query_idx];
		if (struct_idx < 0)				// -1로 값이 없다면 그냥 지나간다 -> 이걸 조금 더 효율적으로 할 수 있을 듯?
		{
			continue;
		}

		object_points.push_back(structure[struct_idx]);		// 이전 point들에 대해 매칭되는 world point를 저장한다
		image_points.push_back(key_points[train_idx].pt);	// 이전 point들에 대해 매칭되는 keyoint를 저장한다
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

	// 현재 비교하는 매치의 모든 개수!! -> 그래서 vector<DMatch> 이거야
	for (int i = 0; i < matches.size(); ++i)
	{
		int query_idx = matches[i].queryIdx;
		int train_idx = matches[i].trainIdx;

		// struct_indices : 이전 sturcture의 idx 정보
		int struct_idx = struct_indices[query_idx];
		if (struct_idx >= 0)	// struct_indices의 값이 유효하다면 struct_idx을 매칭되는 점에 표시!!
		{
			next_struct_indices[train_idx] = struct_idx;
			continue;
		}

		// 매칭이 안된다면 해당 점을 새로운 점으로 추가한다
		// 만약 structure 여기에 next_structure의 i 번째 친구를 추가 저장한다
		structure.push_back(next_structure[i]);
		colors.push_back(next_colors[i]);

		// structure.size() - 1 ( index 새로 저장 )
		struct_indices[query_idx] = next_struct_indices[train_idx] = structure.size() - 1;
	}
}