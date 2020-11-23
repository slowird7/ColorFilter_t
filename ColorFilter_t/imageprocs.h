#pragma once

#include "stdafx.h"
#define _USE_MATH_DEFINES
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <filesystem> 
#include <string>    // useful for reading and writing
#include <fstream>   // ifstream, ofstream
#include <math.h>

using namespace std;
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#pragma comment(lib,"opencv_world341.lib")
#include <mutex>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <strstream>

typedef struct _color_filter {
	double h_min = 150;
	double h_max = 10;
	double s_min = 0;
	double s_max = 255;
	double v_min = 250;
	double v_max = 255;
	int line_length_min = 25;

} color_filter;

extern color_filter g_filter;

typedef struct _contourEx {
	vector<cv::Point> contour;
	int size;
	cv::RotatedRect rect;
} contourEx;

void FitLines(cv::Mat fit, cv::Point& contoursP1, cv::Point& contoursP2, cv::Point& topResult1, cv::Point& topResult2, boolean first, cv::Mat& input_image_rgb);
void FitLines2(cv::Mat fit, cv::Point& contoursP1, cv::Point& contoursP2, cv::Point& topResult1, cv::Point& topResult2, boolean first, cv::Mat& input_image_rgb);
cv::Mat createMask(cv::Mat laser_image_hsv, color_filter filter, boolean fine);
cv::Mat LineMaskForDrawAxis(cv::Mat laser_image_hsv, color_filter filter, double min_contour_size, boolean fine);
cv::Mat LineMaskForDetectSegmentEdge(cv::Mat laser_image_hsv, color_filter filter, double min_contour_size, boolean fine);
int Distance(vector<cv::Vec4i> a, cv::Point& PCross, cv::Point& PSegmentEdge, vector<cv::Vec4i> mask);
void checkDuplicate(cv::Mat input_image, vector<cv::Vec4i>& labels);

cv::Mat filterImageByHSV(cv::Mat input_hsv, color_filter filter);
vector<contourEx> findContourOfLaserReflection(cv::Mat input_hsv, double angle_min, double angle_max);
