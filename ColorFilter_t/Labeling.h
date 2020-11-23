#pragma once
#include<opencv2/opencv.hpp>
using namespace cv;

double Labeling(Mat mask);

double HSVmask();

void filter_green();

void filter_red();

void color_check();

void color_check2();