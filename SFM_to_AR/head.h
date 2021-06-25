#include "cloud.h"

float a = -0.0;
float b = 0.0;
float c = 0.0;
float d = 2;
int t = 0;

void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'a':  a = a - 0.05; break; // 1
	case 'd':  a = a + 0.05; break; // 2

	case 'w':  b = b + 0.05; break; // 3
	case 's':  b = b - 0.05; break; // 4

	case 'q':  c = c + 0.5; break; // 3
	case 'e':  c = c - 0.5; break; // 4

	case '1':  d = d + 0.05; break; // 3
	case '2':  d = d - 0.05; break; // 4

	case '3':  t = t + 1; break; // 3
	case '4':  t = t - 1; break; // 4

	case 'f':  frame = frame + 1; break; // 4
	case 'g':  frame = frame - 1; break; // 4

	case '\x1B':
	{
		exit(EXIT_SUCCESS);
		break;
	}
	}

	cout << "a : " << a << " b : " << b << " c : " << c << " d : " << d << endl;
}

Mat scaled;

void drawOcc() {

	glScissor(0, 0, w, h);
	glViewport(0, 0, w, h);		// 640 x 480

	//glScalef(1, -1, -1);
	// 여기서 회전만 어떻게 잘하면 될 듯 -> 여기서 이미지 계산
	
	Mat posem = Mat::zeros(4, 4, CV_32FC1);

	rotations[frame].convertTo(rotations[frame], CV_32FC1);
	motions[frame].convertTo(motions[frame], CV_32FC1);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			posem.at<float>(i, j) = rotations[frame].at<float>(i, j);
		}
	}

	for (int k = 0; k < 3; k++)
	{
		posem.at<float>(k, 3) = motions[frame].at<float>(k, 0);
	}

	posem.at<float>(3, 3) = 1.f;

	cout << posem.type() << "pose : \n" << posem << endl << endl;

	posem = scaled * posem;

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	transpose(posem, posem);
	memcpy(posem_array, posem.data, sizeof(float) * 16);
	glLoadMatrixf(posem_array);


	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glScalef(1, -1, -1);
	gluPerspective(2.0f * (180 / CV_PI) * atan2(h, 2.f * calib[1]), ((float)w * calib[1]) / ((float)h * calib[1]), 1.0, 200.0);
	glScalef(1, -1, 1);

	glDepthFunc(GL_LEQUAL);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glTranslatef((structure[t].x + a), (structure[t].y + b), (structure[t].z + c));

	cout << "x : " << a + structure[t].x << " y : " << b + structure[t].y << " z : " << structure[t].z + c << " d : " << d << endl;

	glScalef(1, -1, -1);
	glRotatef(-45, 0, 1, 0);
	glRotatef(45, 1, 0, -1);

	// draw_axis(d);
	glColor3f(0.f, 0.f, 1.f);
	glutSolidTeapot(d);

	// 여기 변경하기

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glDepthFunc(GL_ALWAYS);
}

void newdrawOcc() {

	glScissor(0, 0, w, h);
	glViewport(0, 0, w, h);		// 640 x 480

	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			pDepth[i*w + j] = pDepth[i*w + j] / 20;
		}
	}

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glPushMatrix();

	gluOrtho2D(0.0, w, 0.0, h);
	glScalef(1, -1, 1);  // Change window coordinate (y+ = down)
	glTranslatef(0, -h, 0);

	//Draw depth texture for occlusion
	glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);		// 여기가 진짜진짜 중요해!
	glUseProgram(fragmentProgram);

	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, w, h, 0, GL_DEPTH_COMPONENT, GL_FLOAT, pDepth);		// pDepth가 있네?
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f); glVertex2f(0.0f, 0.0f);
	glTexCoord2f(1.0f, 0.0f); glVertex2f(w, 0.0f);
	glTexCoord2f(1.0f, 1.0f); glVertex2f(w, h);
	glTexCoord2f(0.0f, 1.0f); glVertex2f(0.0f, h);
	glEnd();
	glUseProgram(0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();


	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	//glScalef(1, -1, -1);
	// 여기서 회전만 어떻게 잘하면 될 듯 -> 여기서 이미지 계산

	Mat posem = Mat::zeros(4, 4, CV_32FC1);

	rotations[frame].convertTo(rotations[frame], CV_32FC1);
	motions[frame].convertTo(motions[frame], CV_32FC1);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			posem.at<float>(i, j) = rotations[frame].at<float>(i, j);
		}
	}

	for (int k = 0; k < 3; k++)
	{
		posem.at<float>(k, 3) = motions[frame].at<float>(k, 0);
	}

	posem.at<float>(3, 3) = 1.f;

	cout << posem.type() << "pose : \n" << posem << endl << endl;

	posem = scaled * posem;

	transpose(posem, posem);
	memcpy(posem_array, posem.data, sizeof(float) * 16);
	glLoadMatrixf(posem_array);


	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glScalef(1, -1, -1);

	gluPerspective(2.0f * (180 / CV_PI) * atan2(h, 2.f * calib[1]), ((float)w * calib[1]) / ((float)h * calib[1]), 1.0, 200.0);

	glDepthFunc(GL_LEQUAL);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

	glTranslatef((structure[t].x + a), (structure[t].y + b), (structure[t].z + c));

	cout << "x : " << a + structure[t].x << " y : " << b + structure[t].y << " z : " << structure[t].z + c << " d : " << d << endl;

	glRotatef(-45, 0, 1, 0);
	glRotatef(45, 1, 0, -1);

	// draw_axis(d);
	//glColor3f(0.f, 0.f, 1.f);
	glutSolidTeapot(d);

	// 여기 변경하기

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glDepthFunc(GL_ALWAYS);
}

void draw() {
	draw_img();
	drawOcc();

	glFlush();
	glutSwapBuffers();
}

void proc() {

	colorImage = imread(img_names[frame], 1);

	imshow("test", colorImage);
	waitKey(27);

	cvtColor(colorImage, colorImage, CV_RGB2BGR);
	cvtColor(colorImage, grayImage, CV_BGR2GRAY);

	depthImage = cv::imread(depth_names[frame], -1);

	depthImage.convertTo(depthImage, CV_16SC1);

	//depthImage = pull_push(depthImage);
	depthImage.convertTo(depthImage, CV_32FC1);

	normalize(depthImage, depthImage2, 0, 255, NORM_MINMAX, CV_8U);
	imshow("depth", depthImage2);
	waitKey(27);
	// if(frame == 500) waitKey(0);

	// pDepth로 이동. 이건 없어져도 괜찮겠어
	SetImage(colorImage.data);
	SetDepth(depthImage.data);

	//draw();

	//frame++;
	glutPostRedisplay();
}