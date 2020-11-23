#include"stdafx.h"
#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
#include<random>
#include<string>
#include<strstream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;

//���o����摜�̋P�x�l�͈̔͂��w��
constexpr auto Label_H_MAX = 70;
constexpr auto Label_H_MIN = 50;
constexpr auto Label_S_MAX = 255;
constexpr auto Label_S_MIN = 30;
constexpr auto Label_V_MAX = 255;
constexpr auto Label_V_MIN = 60;

int H_MAX = 70;
int S_MAX = 255;
int V_MAX = 255;
int H_MIN = 50;
int S_MIN = 30;
int V_MIN = 60;

cv::Mat Label_LabelImg;
cv::Mat Label_stats;
cv::Mat Label_centroids;

#define length 50.00

double Label_value = 0;

Mat inputImag;
Point point_m;

void filter_green();
void filter_red();
void color_check();
void color_check2();

double Labeling(Mat mask) {
	int Label_num;
	int max = INT_MIN;
	std::array<int, 1000> number;


	//���׃����O����
	int nLab = cv::connectedComponentsWithStats(mask, Label_LabelImg, Label_stats, Label_centroids);

	// ���x�����O���ʂ̕`��F������
	std::vector<cv::Vec3b> colors(nLab);
	colors[0] = cv::Vec3b(0, 0, 0);
	for (int i = 1; i < nLab; ++i) {
		colors[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
	}

	// ���x�����O���ʂ̕`��
	cv::Mat Dst(mask.size(), CV_8UC3);
	for (int i = 0; i < Dst.rows; ++i) {
		int *lb = Label_LabelImg.ptr<int>(i);
		cv::Vec3b *pix = Dst.ptr<cv::Vec3b>(i);
		for (int j = 0; j < Dst.cols; ++j) {
			pix[j] = colors[lb[j]];
		}
	}

	//ROI�̐ݒ�(�Ώۗ̈�)
	for (int i = 1; i < nLab; ++i) {
		int *param = Label_stats.ptr<int>(i);

		int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

		cv::rectangle(Dst, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 2);
	}

	//�d�S�̏o��
	for (int i = 1; i < nLab; ++i) {
		double *param = Label_centroids.ptr<double>(i);
		int x = static_cast<int>(param[0]);
		int y = static_cast<int>(param[1]);

		cv::circle(Dst, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
	}

	//�ʐϒl�̏o�́E���������̏o��
	for (int i = 1; i < nLab; ++i) {
		int *param = Label_stats.ptr<int>(i);

		//���ʐ�(�s�N�Z���P��)100�ȉ��͖�������
		if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA] > 100) {
			number[i] = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];

			//std::cout << "�������� : " << i << " = " << param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT] << std::endl;
			std::cout << "�������� : " << i << " = " << param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH] << std::endl;
			//std::cout << "���ʐ� : " << i << " = " << param[cv::ConnectedComponentsTypes::CC_STAT_AREA] << std::endl;

			//ROI�̍���ɔԍ�����������
			int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
			int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
			std::stringstream num;
			num << i;
			cv::putText(Dst, num.str(), cv::Point(x + 5, y + 20), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
			
			if (number[i] > max) {
				max = std::max(max, number[i]);
				Label_num = i;
			}
			else {
				//Label_num = numberi;
			}
		}

	}

	cv::imwrite("labeling.png", Dst);
	//imshow("labeling", Dst);
	printf("labeling.png���o�͂��܂����B\n");

	//CorrectionValu�͂����Ɉړ�
	printf("Value�J�n\n");
	
	int *param = 0;
	int pixcel = 0;

	param = Label_stats.ptr<int>(Label_num);
	pixcel = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];

	Label_value = floor(100 * (length / pixcel)) / 100;

	printf("�␳�l = %f\n", Label_value);

	return Label_value;
}

//HSV�ɂ��F���o
double HSVmask() {
	printf("HSVmask���s\n");

	//���͉摜

	//�摜��3�`�����l��(BGR)�œǂݍ���
	Mat input_image_rgb = inputImag;
	if (input_image_rgb.empty()) {
		cerr << "���͉摜��������܂���B" << endl;
		return -1;
	}

	//BGR����HSV�֕ϊ�
	Mat hsv_image, mask_image, output_image;
	cvtColor(input_image_rgb, hsv_image, CV_BGR2HSV, 3);

//	color_check2();
	color_check();

	//inRange��p���ăt�B���^�����O
	//Scalar s_min = Scalar(Label_H_MIN, Label_S_MIN, Label_V_MIN);
	//Scalar s_max = Scalar(Label_H_MAX, Label_S_MAX, Label_V_MAX);
	Scalar s_min = Scalar(H_MIN, S_MIN, V_MIN);
	Scalar s_max = Scalar(H_MAX, S_MAX, V_MAX);
	inRange(hsv_image, s_min, s_max, mask_image);

	//�}�X�N�摜
	imwrite("Label_mask.png", mask_image);

	//�}�X�N�����Ƃɓ��͉摜���t�B���^�����O
	input_image_rgb.copyTo(output_image, mask_image);

	//���ʂ̕ۑ�
	imwrite("Label_output.png", output_image);

	double mask_image_pix = 0;

	mask_image_pix = Labeling(mask_image);
	

	return mask_image_pix;
}

void filter_green() {

	//�Ǎ���
	cv::Mat srcimg = inputImag;
	cv::Mat ret_mat, hsv_image, dst_img, mask_image;

	//HSV�֕ϊ�
	cv::cvtColor(srcimg, hsv_image, CV_BGR2HSV);
	std::vector<cv::Mat> mat_channels;
	cv::split(hsv_image, mat_channels);

	int hue_min = 50;
	int hue_max = hue_min + 20;

	int hsv_value;
	//�s:cols�E��:rows
	int cols = hsv_image.cols;
	int rows = hsv_image.rows;

	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			hsv_value = static_cast<int>(mat_channels[0].at<uchar>(row, col));
				if (!((hue_min <= hsv_value) && (hsv_value <= hue_max))) {
				mat_channels[1].at<uchar>(row, col) = 0;
			}
		}
	}

	
	cv::merge(mat_channels, dst_img);
	cv::cvtColor(dst_img, ret_mat, CV_HSV2BGR);
	imwrite("test_hsv.png", dst_img);
	imwrite("test_out.png", ret_mat);

	return;
}

void filter_red() {
	cv::Mat srcimg = inputImag;
	cv::Mat ret_mat, hsv_image, dst_img;

	cv::cvtColor(srcimg, hsv_image, CV_BGR2HSV);
	std::vector<cv::Mat> mat_channels;
	cv::split(hsv_image, mat_channels);

	int line_min = 160;
	int line_max = line_min + 20;

	int hsv_value;
	int cols = hsv_image.cols;
	int rows = hsv_image.rows;

	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			hsv_value = static_cast<int>(mat_channels[0].at<uchar>(row, col));
			if (!((line_min <= hsv_value) && (hsv_value <= line_max))) {
				mat_channels[1].at<uchar>(row, col) = 0;
			}
		}
	}

	cv::merge(mat_channels, dst_img);
	cv::cvtColor(dst_img, ret_mat, CV_HSV2BGR);
	imwrite("test_hsv2.png", dst_img);
	imwrite("test_out2.png", ret_mat);

	return;
}

void color_check(){
	//HSV�p�����[�^�[
	double sikisou, saido, meido;

	//���W���w��
	int x = point_m.x;
	int y = point_m.y;

	cv::Mat srcimg = inputImag;
	cv::Mat3b dotimg = srcimg;

	//rgb�m�F
	cv::Vec3b bgr = dotimg(cv::Point(x, y));
	//printf("B[%d] G[%d] R[%d]\n", bgr[0], bgr[1], bgr[2]);

	double R = bgr[2];
	double G = bgr[1];
	double B = bgr[0];

	double MAX = max(max(R, G), B);
	double MIN = min(min(R, G), B);
	meido = MAX / 256 * 100;

	if (MAX == MIN) {
		sikisou = 0;
		saido = 0;
	}
	else {
		if (MAX == R) sikisou = 60.0*(G - B) / (MAX - MIN) + 0;
		else if (MAX == G) sikisou = 60.0*(B - R) / (MAX - MIN) + 120.0;
		else if (MAX == B) sikisou = 60.0*(R - G) / (MAX - MIN) + 240.0;

		if (sikisou > 360.0) sikisou = sikisou - 360.0;
		else if (sikisou < 0) sikisou = sikisou + 360.0;
		saido = (MAX - MIN) / MAX * 100;
	}
	printf("�F��[%.2f] �ʓx[%.2f] ���x[%.2f]\n", sikisou, saido, meido);

	S_MIN = saido;
	V_MIN = meido;
	H_MAX = sikisou - 40;

	return;
}

void color_check2() {
	//HSV�p�����[�^�[
	double sikisou, saido, meido;

	//���W���w��
	int x = 509;
	int y = 424;

	cv::Mat srcimg = inputImag;
	cv::Mat3b dotimg = srcimg;

	//rgb�m�F
	cv::Vec3b bgr = dotimg(cv::Point(x, y));
	//printf("���C�����[�U�[�@B[%d] G[%d] R[%d]\n", bgr[0], bgr[1], bgr[2]);

	double R = bgr[2];
	double G = bgr[1];
	double B = bgr[0];

	double MAX = max(max(R, G), B);
	double MIN = min(min(R, G), B);
	meido = MAX / 256 * 100;

	if (MAX == MIN) {
		sikisou = 0;
		saido = 0;
	}
	else {
		if (MAX == R) sikisou = 60.0*(G - B) / (MAX - MIN) + 0;
		else if (MAX == G) sikisou = 60.0*(B - R) / (MAX - MIN) + 120.0;
		else if (MAX == B) sikisou = 60.0*(R - G) / (MAX - MIN) + 240.0;

		if (sikisou > 360.0) sikisou = sikisou - 360.0;
		else if (sikisou < 0) sikisou = sikisou + 360.0;
		saido = (MAX - MIN) / MAX * 100;
	}
	printf("���C�����[�U�[�@�F��[%.2f] �ʓx[%.2f] ���x[%.2f]\n", sikisou, saido, meido);

	return;
}