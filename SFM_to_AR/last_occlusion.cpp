#include "head.h"

int main(int argc, char **argv) {

	scaled = Mat::eye(4, 4, CV_32FC1);

	scaled.at<float>(1, 1) = -1.0;
	scaled.at<float>(2, 2) = -1.0;

	dir = "./img/";

	cameraInit();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);

	glutInitWindowSize(w, h);
	glutInitWindowPosition(w / 2, h / 2);
	glutCreateWindow("AR Occlusion");

	init();

	proc();

	glutDisplayFunc(draw);
	glutKeyboardFunc(keyboard);

	glutIdleFunc(proc);

	glutMainLoop();

	return EXIT_SUCCESS;
}