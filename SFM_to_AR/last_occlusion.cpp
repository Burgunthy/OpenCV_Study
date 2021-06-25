#include "head.h"

void jsonWrite(int i) {

	rotations[i].convertTo(rotations[i], CV_32FC1);
	motions[i].convertTo(motions[i], CV_32FC1);

	ofstream json_file; json_file.open(string_format("./pose/%04d.json", i));

	Json::Value root;
	root["c_y"] = calib[3];
	root["c_x"] = calib[2];

	Json::Value extrinsic;

	Json::Value extrinsic1;
	Json::Value extrinsic2;
	Json::Value extrinsic3;
	Json::Value extrinsic4;

	extrinsic1.append(rotations[i].at<float>(0, 0));
	extrinsic1.append(rotations[i].at<float>(0, 1));
	extrinsic1.append(rotations[i].at<float>(0, 2));
	extrinsic1.append(motions[i].at<float>(0, 0));

	extrinsic2.append(rotations[i].at<float>(1, 0));
	extrinsic2.append(rotations[i].at<float>(1, 1));
	extrinsic2.append(rotations[i].at<float>(1, 2));
	extrinsic2.append(motions[i].at<float>(1, 0));

	extrinsic3.append(rotations[i].at<float>(2, 0));
	extrinsic3.append(rotations[i].at<float>(2, 1));
	extrinsic3.append(rotations[i].at<float>(2, 2));
	extrinsic3.append(motions[i].at<float>(2, 0));

	extrinsic4.append(0);
	extrinsic4.append(0);
	extrinsic4.append(0);
	extrinsic4.append(1);

	extrinsic.append(extrinsic1);
	extrinsic.append(extrinsic2);
	extrinsic.append(extrinsic3);
	extrinsic.append(extrinsic4);

	root["extrinsic"] = extrinsic;

	root["f_x"] = calib[0];
	root["f_y"] = calib[1];

	Json::StreamWriterBuilder builder;
	builder["commentStyle"] = "None";
	builder["indentation"] = "	"; // Tab
	
	unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());

	// 알파벳 순으로 write 된다.
	writer->write(root, &cout);
	writer->write(root, &json_file);
	cout << endl; // add lf and flush 
	
	json_file.close();
}

int main(int argc, char **argv) {

	scaled = Mat::eye(4, 4, CV_32FC1);

	scaled.at<float>(1, 1) = -1.0;
	scaled.at<float>(2, 2) = -1.0;

	dir = "M:/drive/Etri_Code/dataset/making_depth_data/original_test/";

	cameraInit();
	
	for (int i = 0; i < rotations.size(); i++) {
		jsonWrite(i);
	}

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