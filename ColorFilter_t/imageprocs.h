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
using namespace cv;
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

void FitLines(Mat fit, Point& contoursP1, Point& contoursP2, Point& topResult1, Point& topResult2, boolean first, Mat& input_image_rgb);
void FitLines2(Mat fit, Point& contoursP1, Point& contoursP2, Point& topResult1, Point& topResult2, boolean first, Mat& input_image_rgb);
Mat createMask(Mat laser_image_hsv, color_filter filter, boolean fine);
Mat LineMaskForDrawAxis(Mat laser_image_hsv, color_filter filter, double min_contour_size, boolean fine);
Mat LineMaskForDetectSegmentEdge(Mat laser_image_hsv, color_filter filter, double min_contour_size, boolean fine);
int Distance(vector<Vec4i> a, Point& PCross, Point& PSegmentEdge, vector<Vec4i> mask);
void checkDuplicate(Mat input_image, vector<Vec4i>& labels);
