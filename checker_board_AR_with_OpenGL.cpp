#include <GL/freeglut.h>
#include "opencv2/opencv.hpp" 
#include <opencv2/calib3d.hpp> 
#include <iostream>  
#include <string> 

using namespace cv;
using namespace std;

VideoCapture * capture;
Mat img_cam;
int width = 640;
int height = 480;
Mat mask = Mat(height, width, CV_8UC3, Scalar(0, 0, 0));
GLuint texture_background, texture_cube;

/* Camera Calbration Variable -- New One */
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


vector<Point3f> objectPoints;	// 3d world coordinates
vector<Point2f> imagePoints;	// 2d image coordinates
bool patt_found = false;

Size board_size = Size(8, 6);
float* pose = new float[16];

Mat cam_mat = Mat(3, 3, CV_64FC1, Scalar(0.));
Mat mv_mat = Mat(4, 4, CV_64FC1, Scalar(0.));
Mat dist_coeffs = Mat(1, 4, CV_64FC1, Scalar(0.));
Mat rot;
Mat rot_rod;
Mat trans;
Mat dist_coeffs_double;

// 카메라 초기화
void cameraInit()
{
	capture = new VideoCapture(1);

	if (!capture) {
		printf("Could not capture a camera\n\7");
		return;
	}

	Mat img_frame;

	capture->read(img_frame);

	width = img_frame.cols / 2;
	height = img_frame.rows;

	cout << width << " " << height << endl;

	cam_mat.at<double>(0, 0) = fxL;
	cam_mat.at<double>(0, 2) = cxL;
	cam_mat.at<double>(1, 1) = fyL;
	cam_mat.at<double>(1, 2) = cyL;
	cam_mat.at<double>(2, 2) = 1;

	dist_coeffs.at<double>(0, 0) = k1L;
	dist_coeffs.at<double>(0, 1) = k2L;
	dist_coeffs.at<double>(0, 2) = p1L;
	dist_coeffs.at<double>(0, 3) = p2L;

	for (int i = 0; i < board_size.height; i++) {
		for (int j = 0; j < board_size.width; j++) {
			objectPoints.push_back(Point3f((j * 0.03f), (i * 0.03f), 0));
		}
	}
}

void init()
{
	// clear background color
	glClearColor(0.f, 0.f, 0.f, 0.f);

	//깊이 버퍼 활성화
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_CULL_FACE);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
	glLineWidth(5.0f);

}

void draw_axis(void)
{
	float axis_scale = 0.06f;
	glLineWidth(3);
	glBegin(GL_LINES);

	glColor3f(1.f, 0.f, 0.f);
	glVertex3f(0.f, 0.f, 0.f);
	glVertex3f(axis_scale, 0.f, 0.f);

	glColor3f(0.f, 1.f, 0.f);
	glVertex3f(0.f, 0.f, 0.f);
	glVertex3f(0.f, axis_scale, 0.f);

	glColor3f(0.f, 0.f, 1.f);
	glVertex3f(0.f, 0.f, 0.f);
	glVertex3f(0.f, 0.f, axis_scale);

	glEnd();
	glLineWidth(1);
}

void draw()
{
	cout << "7" << endl;

	//화면을 지운다. (컬러버퍼와 깊이버퍼)
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	if (patt_found) {

		glPushMatrix();

		glLoadMatrixf(pose);
		glRotatef(90, 1, 0, 0);
		// glutSolidTeapot(4.f);
		draw_axis();

		glColor3f(1.f, 0.f, 1.f);
		glTranslatef(0.03f * 4, 0, 0);
		glutSolidTeapot(0.03f);

		glColor3f(1.f, 1.f, 0.f);
		glTranslatef(0, 0, 0.03f * 3);
		glutSolidTeapot(0.03f);
		
		glPopMatrix();
	}

	glFlush();
	glutSwapBuffers();
}

void resize(int width, int height)
{
	cout << "16" << endl;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(2.0f * (180 / 3.141592) * atan2(height, 2.f * fyL), ((float)width * fyL) / ((float)height * fxL), 0.1, 10000.0);
	glViewport(0, 0, width, height);
	// glRotatef(180, 0.0f, 0.0f, 1.0f);

	glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int x, int y)
{
	//ESC 키가 눌러졌다면 프로그램 종료
	if (key == 27)
	{
		capture->release();
		exit(0);
	}
}

void timer(int value) {
	//웹캠으로부터 이미지 캡처
	capture->read(img_cam);
	img_cam = img_cam(cv::Rect(0, 0, width, height)).clone();

	glReadPixels(0, 0, width, height, GL_BGR_EXT, GL_UNSIGNED_BYTE, mask.data);

	/*for (int i = 0; i < width * height; i++) {
		if (mask.data[3 * i] != 0 || mask.data[3 * i + 1] != 0 || mask.data[3 * i + 2] != 0) {
			img_cam.data[3 * i] = mask.data[3 * i];
			img_cam.data[3 * i + 1] = mask.data[3 * i + 1];
			img_cam.data[3 * i + 2] = mask.data[3 * i + 2];
		}
	}*/
	int idx;
	int idx2;

	patt_found = findChessboardCorners(img_cam, board_size, imagePoints);

	if (patt_found) {

		drawChessboardCorners(img_cam, board_size, imagePoints, patt_found);

		solvePnP(objectPoints, imagePoints, cam_mat, dist_coeffs_double, rot, trans);

		rot.at<double>(0, 0) = -rot.at<double>(0, 0);
		trans.at<double>(1, 0) = -trans.at<double>(1, 0);
		trans.at<double>(2, 0) = -trans.at<double>(2, 0);

		Rodrigues(rot, rot_rod);

		/// Camera extrinsic matrix
		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++) {
				mv_mat.at<double>(row, col) = rot_rod.at<double>(row, col);
			}
			mv_mat.at<double>(row, 3) = trans.at<double>(row, 0);
		}
		mv_mat.at<double>(3, 3) = 1.0;

		// make transposed matrix
		for (int i = 0; i < 3; i++) {
			double tmp = mv_mat.at<double>(i, 3);
			mv_mat.at<double>(i, 3) = mv_mat.at<double>(3, i);
			mv_mat.at<double>(3, i) = tmp;
		}

		// Save extrinsic parameters into array
		for (int row = 0; row < 4; row++) {
			for (int col = 0; col < 4; col++) {
				pose[row * 4 + col] = (float)mv_mat.at<double>(row, col);
			}
		}

#pragma omp parallel for
		for (int i = 0; i < width; i++) {
			for (int j = 0; j < height; j++) {
				idx = i + j * width;
				idx2 = i + (height - 1 - j) * width;

				if (mask.data[3 * idx] != 0 || mask.data[3 * idx + 1] != 0 || mask.data[3 * idx + 2] != 0) {
					img_cam.data[3 * idx2] = mask.data[3 * idx];
					img_cam.data[3 * idx2 + 1] = mask.data[3 * idx + 1];
					img_cam.data[3 * idx2 + 2] = mask.data[3 * idx + 2];
				}
			}
		}
	}

	imshow("src", img_cam);

	cout << "2222" << endl;

	glutPostRedisplay();      //윈도우를 다시 그리도록 요청
	glutTimerFunc(1, timer, 0); //다음 타이머 이벤트는 1밀리세컨트 후  호출됨.
}

/* Main method */
int main(int argc, char** argv)
{
	// glut initialize
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

	// 카메라 시작하기
	cameraInit();

	glutInitWindowSize(width, height);
	glutInitWindowPosition(width / 2, height / 2);
	glutCreateWindow("GLUT Test");	// 위의 순서 무조건 고정

	// 사용자 초기화 함수
	init();

	/* Create a single window with a keyboard and display callback */
	glutKeyboardFunc(&keyboard);

	glutDisplayFunc(&draw);
	glutReshapeFunc(&resize);
	glutTimerFunc(0, timer, 0);		// 모든 함수를 다 저장한다

	/* Run the GLUT event loop */
	glutMainLoop();		// 여기서 모든게 시작되는 것 같아

	return EXIT_SUCCESS;
}

