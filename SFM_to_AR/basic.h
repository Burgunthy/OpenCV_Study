#define _CRT_SECURE_NO_WARNINGS

#include <GL/glew.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <iostream>
#include <vector>
#include <io.h>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

#define num 30

int frame = 0;
string dir;

int w = 640;
int h = 480;
Size frameSize = Size(w, h);

double calib[4] = { 525.0 , 525.0, 319.5, 239.5 };
//double calib[4] = { 600.0 , 600.0, 320.0, 240.0 };
//double calib[4] = { 2759.48 , 2764.16, 1520.69, 1006.81 };

Mat K(Matx33d(
	calib[0],	0,			calib[2],
	0,			calib[1],	calib[3],
	0,			0,			1
));

Mat colorImage = cv::Mat::zeros(frameSize, CV_8UC3);
Mat depthImage = cv::Mat::zeros(frameSize, CV_16SC1);
Mat grayImage = cv::Mat::zeros(frameSize, CV_8UC1);

// Occlusion Aware
GLuint colorTexture, depthRenderBuffer;
GLuint depthTexture = 0;
GLuint frameBuffer = 0;
GLuint fragmentProgram;
GLuint buffer_id;
GLuint fragmentShader;
GLubyte *ptr;

float posem_array[16];

unsigned char *pColor;
float *pDepth;
float *depthVertices;

// SFM code
vector<Mat> rotations;						// 이미지 당 모든 rot
vector<Mat> motions;
vector<Point3f> structure;					// 전체 point의 위치 정보

void initOcc() {
	const GLchar *shaderSource[] = { "uniform sampler2D depthMap;void main(){gl_FragDepth = texture2D(depthMap, gl_TexCoord[0].st).r;}" };		// 셰이더 선언
	fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);				// fragment shader를 사용한다
	glShaderSource(fragmentShader, 1, (const GLchar**)&shaderSource, NULL);			// glShaderSource에 연결
	// void glShaderSource(GLuint shader, GLsizei count, const GLchar** string, const GLint* length);
	// shader : GLSL 소스를 등록할 셰이더 객체
	// count : 문자열들의 갯수
	// string : 문자열들의 배열
	// length : 문자열들의 길이 배열

	glCompileShader(fragmentShader);
	// 셰이더 객체에 들어있는 GLSL 소스를 컴파일합니다.
	GLint isCompiled = 0;
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &isCompiled);
	// glGetShaderiv(GLuint shader, GLenum pname, GLint *params)
	// 셰이더의 정보를 가져옵니다.
	//GL_COMPILE_STATUS : 셰이더 컴파일이 성공적으로 이루어 졌는지
	//위 두 값은 성공적이었다면 GL_TRUE 가 params 에 저장되며 그렇지 않은 경우 GL_FASE가 params에 저장됩니다.

	fragmentProgram = glCreateProgram();
	// 사용 가능한 프로그램 객체의 ID를 하나 반환
	glAttachShader(fragmentProgram, fragmentShader);
	// 프로그램에 셰이더를 첨부
	glLinkProgram(fragmentProgram);
	// 프로그램 객체를 연결
	GLint isLinked = 0;
	glGetProgramiv(fragmentProgram, GL_LINK_STATUS, (int *)&isLinked);
	// 프로그램의 정보를 가져옴

	pDepth = new float[w*h];
	depthVertices = new float[w*h];
}

void init() {
	glEnable(GL_DEPTH_TEST);
	glDepthMask(GL_TRUE);
	glDepthFunc(GL_ALWAYS);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);

	//RGB Init
	glGenTextures(1, &colorTexture);
	glBindTexture(GL_TEXTURE_2D, colorTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//Depth Init
	glGenTextures(1, &depthTexture);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_NONE);
	glBindTexture(GL_TEXTURE_2D, 0);

	glewInit();
	initOcc();
}

// 우선 이미지를 복사한다
void draw_img() {

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	glPushMatrix();
	gluOrtho2D(0.0, w, 0.0, h);

	glScalef(1, -1, 1);  // Change window coordinate (y+ = down)
	glTranslatef(0, -h, 0);

	glClearColor(1.f, 1.f, 1.f, 0.f);
	glScissor(0, 0, w, h);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glViewport(0, 0, w, h);

	//draw_triangle();

	glBindTexture(GL_TEXTURE_2D, colorTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, pColor);
	glBegin(GL_TRIANGLE_FAN);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glTexCoord2f(0, 0);  glVertex3f(0, 0, 0);
	glTexCoord2f(1, 0);  glVertex3f(w, 0, 0);
	glTexCoord2f(1, 1);  glVertex3f(w, h, 0);
	glTexCoord2f(0, 1);  glVertex3f(0, h, 0);
	glEnd();
	glBindTexture(GL_TEXTURE_2D, 0);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

void SetImage(unsigned char *input)
{
	pColor = input;
}

void SetDepth(unsigned char *depth)
{
	memcpy(pDepth, depth, sizeof(float)*w*h);
	//memcpy(pDepth, depth, sizeof(uchar)*w*h);
}

template<typename ... Args>
std::string string_format(const std::string& format, Args ... args) {
	size_t size = snprintf(nullptr, 0, format.c_str(), args ...) + 1;
	if (size <= 0) { throw std::runtime_error("Error during formatting."); }
	std::unique_ptr<char[]> buf(new char[size]); snprintf(buf.get(), size, format.c_str(), args ...);
	return std::string(buf.get(), buf.get() + size - 1);
}