#include "stdafx.h"
#define _USE_MATH_DEFINES
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <filesystem> 
#include <string>    // useful for reading and writing
#include <fstream>   // ifstream, ofstream
#include <math.h>

#pragma comment(lib,"opencv_world341.lib")
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <mutex>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <vector>
#include <random>
#include <strstream>
#include "imageprocs.h"

using namespace std;
using namespace cv;

void mark_check(int r, int g, int b);
double Labeling(Mat mask);
void HSVmask();

//マーキングテープを抽出する画像の輝度値の範囲を指定
int MarkH_MAX = 70;
int MarkS_MAX = 255;
int MarkV_MAX = 255;
int MarkH_MIN = 30;
int MarkS_MIN = 120;
int MarkV_MIN = 120;

cv::Mat Label_LabelImg;
cv::Mat Label_stats;
cv::Mat Label_centroids;

#define Label_length 50.00
#define SPREADER_SIZE 200  // [mm]

double Label_value = 0;
//ここまでラベリング処理で使用


void Calculation(int diff);

// 抽出する画像の輝度値の範囲を指定
#define H_MAX 120
#define H_MIN 60
#define S_MAX 255
#define S_MIN 105
#define V_MAX 255
#define V_MIN 100


//補正
int center_X, mark_X = 0;


//線が検出できたかのフラグできていたらfalse
bool zeroCheck_T = false;
bool zerocheck_U = false;
int zeroCheck1 = 0, zeroCheck2 = 0;
bool error_F = false;

bool first = true;

//extern Mat inputImag;
extern Point point_m;


//直線検出用
std::vector<Vec4i> Line;

std::vector<Vec4i> topLine, underLine;

std::vector<Vec2f> lines;


//比較物のピクセル数
#define Pixel 48.00
//比較物の長さ
#define length 50.00

Mat input_image_rgb, hsv_image, laser_image_hsv, mask_image, output_image;
Mat stats, centroids, mask_image2, output_image2;
Mat laserOFF_image, laser_image, result_l;
Mat thin_image, ans_image, result_i;
vector<Vec4i> labels;
String inputFile, inputFile2, inputFilter;
double Label_pix, result = 0;
int filtersize;
Point contoursP1, contoursP2, topResult1, topResult2;
Point PCross, PSegmentEdge;

bool switching;
HINSTANCE hinstance, hprevinstance;
LPSTR lpszcmdline;
int ncmdshow;
string day, position, cameranuber;
std::vector<Vec4i> output;

void usage() {
	puts("Usage: ColorFilter_t input_image [h_min [h_max]]");
	puts("    h_min : 色相(Hue)「下限値」 (120～179)");
	puts("    h_max : 色相(Hue)「上限値」 (0～60)");

}

char* xgetCurrentPath()
{
	static char path[MAX_PATH];   //パス取得用
	if (::GetModuleFileNameA(NULL, path, MAX_PATH))    //実行ファイルのフルパスを取得
	{   //取得に成功
		char* ptmp = strrchr(path, '\\'); // \の最後の出現位置を取得
		if (ptmp != NULL)
		{   //ファイル名を削除
			ptmp++;   //一文字進める
			*ptmp = _T('\0');
		}
		else
		{
			//エラー：\が無い
		}
	}
	else
	{
		//エラー：取得に失敗
	}
	return path;
}

int loadParameter()
{
	std::ifstream ifs(inputFilter);
	std::string str;
	std::string line;

	if (ifs.fail()) {
		std::cerr << "File Open Error1" << std::endl;
		return -1;
	}
	getline(ifs, line); g_filter.h_min = strtol(line.c_str(), NULL, 10);
	getline(ifs, line); g_filter.h_max = strtol(line.c_str(), NULL, 10);
	getline(ifs, line); g_filter.s_min = strtol(line.c_str(), NULL, 10);
	getline(ifs, line); g_filter.s_max = strtol(line.c_str(), NULL, 10);
	getline(ifs, line); g_filter.v_min = strtol(line.c_str(), NULL, 10);
	getline(ifs, line); g_filter.v_max = strtol(line.c_str(), NULL, 10);
	getline(ifs, line); g_filter.line_length_min = strtol(line.c_str(), NULL, 10);
	ifs.close();
	return 0;
}

int saveParameter(boolean first)
{
	if (first)
	{
		std::ofstream ofs("topParam.txt");
		std::string str;
		std::string line;
		if (ofs.fail()) {
			std::cerr << "File Open Error2" << std::endl;
			return -1;
		}
		ofs << g_filter.h_min << endl;
		ofs << g_filter.h_max << endl;
		ofs << g_filter.s_min << endl;
		ofs << g_filter.s_max << endl;
		ofs << g_filter.v_min << endl;
		ofs << g_filter.v_max << endl;
		ofs << g_filter.line_length_min << endl;
		ofs.close();
	}
	else
	{
		std::ofstream ofs("underParam.txt");
		std::string str;
		std::string line;

		if (ofs.fail()) {
			std::cerr << "File Open Error3" << std::endl;
			return -1;
		}
		ofs << g_filter.h_min << endl;
		ofs << g_filter.h_max << endl;
		ofs << g_filter.s_min << endl;
		ofs << g_filter.s_max << endl;
		ofs << g_filter.v_min << endl;
		ofs << g_filter.v_max << endl;
		ofs << g_filter.line_length_min << endl;
		ofs.close();
	}


	return 0;
}




/**
*
* @parameter
*   (p[0], p[1]) : 始点の座標
*   (p[2], p[3]) : 終点の座標
*
* @return
*   0=対象外 / 1=
*/
int select(Vec4i p) {



	double dX1, dY1, dX2, dY2, inclination1 = 0, inclination2 = 0;




	//とりあえず上側を削除
	if (p[3] < ((input_image_rgb.rows / 2) - 100))
	{
		return 0;
	}

	//中央付近にある？
	if (p[3] < ((input_image_rgb.rows / 2) + 200) || p[1] > ((input_image_rgb.rows / 2) - 200))
	{

		//傾き

		if (p[0] > p[2]) {

			int tmp = p[0];
			p[0] = p[2];
			p[2] = p[0];

			tmp = p[1];
			p[1] = p[3];
			p[3] = tmp;
		}


		dX1 = p[2] - p[0];
		dY1 = p[3] - p[1];

		dX2 = p[0] - p[2];
		dY2 = p[1] - p[3];

		inclination1 = atan2(dY1, dX1);
		inclination2 = atan2(dY2, dX2);

		//許容値をいくつにするか0.01くらい？
		if (inclination2 > 3.105 && inclination2 < 3.159)
		{
			return 1;
		}
		else if (inclination1 > -0.440 && inclination1 < -0.370)
		{
			return 2;
		}
	}
	return 0;

}

std::vector<Vec4i> pickupLine(vector<Vec4i> top, vector<Vec4i> under)
{
	std::vector<Vec4i> result;
	int i = 0;


	//上選定
	if (switching == true) {

		while (top.size() > 1)
		{
			if ((mark_X - center_X) < top[i][0])
			{

				if ((mark_X - center_X) < top[i + 1][0]) {

					if (top[i][0] > top[i + 1][0]) {
						top.erase(top.begin() + i);
					}
					else {
						top.erase(top.begin() + (i + 1));
					}
				}
				else {
					top.erase(top.begin() + (i + 1));
				}
			}
			else {
				if ((mark_X - center_X) > top[i + 1][2]) {
					top.erase(top.begin() + i + 1);
				}

				top.erase(top.begin() + i);
			}

		}
	}
	else {
		if (topResult1.x == 0) {
			printf("セグメント側の該当線が見つかりません\n");
			
		}
		else {
			top = { {topResult1.x,topResult1.y,topResult2.x,topResult2.y} };
		}

	}
	if (top.size() == 0){
		zeroCheck_T = true;
	}else{
		result.push_back(top[0]);
	}




	//スキンプレート側の選定

	if (switching == true) {
		while (under.size() > 1)
		{

			if (top.size() == 0 || top[0][0] > under[i][3])
			{
				if (top.size() == 0 || top[0][0] > under[i + 1][3])
				{



					if (input_image_rgb.rows / 2 < under[i][3])
					{


						if (under[i][2] < under[i + 1][3])
						{
							under.erase(under.begin() + i);
						}
						else
						{
							under.erase(under.begin() + (i + 1));
						}
					}
					else if (input_image_rgb.rows / 2 > under[i + 1][3])
					{
						under.erase(under.begin() + 1);
					}
					else
					{
						under.erase(under.begin());
					}
				}
				else {
					under.erase(under.begin() + 1);
				}
			}
			else {
				under.erase(under.begin());
			}

		}
	}
	else {
		if (contoursP1.x == 0) {
			printf("スキンプレート側に該当する線が見つかりません\n");
		}
		else {
			under = { {contoursP1.x,contoursP1.y,contoursP2.x,contoursP2.y} };
		}
	}

	if (under.size() == 0)
	{
		zerocheck_U = true;
	}
	else
	{
		result.push_back(under[0]);

	}


	//選ばれた線のみ見える
	for (int y = 0; y < result.size(); y++)
	{
		line(result_i, Point(result[y][0], result[y][1]), Point(result[y][2], result[y][3]), Scalar(255, 255, 0), 1, CV_AA);
		
	}


	//線が検出できなかった時の処理
	if (zeroCheck_T && zerocheck_U)
	{
		printf("線が検出できませんでした。\n");
		error_F = true;
	}
	else if (zeroCheck_T)
	{
		printf("上の線が検出できませんでした。\n");
		error_F = true;
	}
	else if (zerocheck_U)
	{
		printf("下の線が検出できませんでした。\n");
		error_F = true;
	}
	//else
	//{
	//	//問題なければ交点計算へ
	//	Distance(result);
	//}
	return result;
}

void detectionLine()
{


	//Hough変換
	//直線の検出

	Mat test;
	input_image_rgb.copyTo(test);

	HoughLinesP(thin_image, Line, 1, CV_PI / 90, g_filter.line_length_min, 25, 20);
	//	HoughLines(thin_image, lines, 1, CV_PI / 90,line_length_min, 25, 25);
	std::vector<Vec2f> lines;
	//HoughLines(input_image, lines, 1, CV_PI / 360, 400, 0, 0);

	/*std::vector<cv::Vec2f>::iterator it = lines.begin();
	for (; it != lines.end(); ++it) {
		float rho = (*it)[0], theta = (*it)[1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cv::saturate_cast<int>(x0 + 1000 * (-b));
		pt1.y = cv::saturate_cast<int>(y0 + 1000 * (a));
		pt2.x = cv::saturate_cast<int>(x0 - 1000 * (-b));
		pt2.y = cv::saturate_cast<int>(y0 - 1000 * (a));
		cv::line(test, pt1, pt2, cv::Scalar(0, 0, 255), 1, CV_AA);
	}*/

//	cv::imwrite("TESTLINE.png", test);
	//すべてのラインを出力する
	/*output_image.setTo(Scalar(0, 0, 0));
	input_image_rgb.copyTo(output_image);
	input_image_rgb.copyTo(result_i);*/

	int check, tcount = 0, ucount = 0;
	for (size_t i = 0; i < Line.size(); ++i)
	{
		float rho = Line[i][0], theta = Line[i][1];

		check = select(Line[i]);

		//条件に会わなければ0が返ってくる
		//selectから返ってきたときの処理
		if (check == 1) {
			bool o = topLine.empty();
			if (switching == true) {
				topLine.push_back(Line[i]);
				output.push_back(Line[i]);
				tcount++;
			}
		}
		else if (check == 2)
		{
			if (switching == true){
				underLine.push_back(Line[i]);
					output.push_back(Line[i]);
					ucount++;
			}

		}





	}

	//検出された線
	if (first == false) {
		for (int p = 0; p < output.size(); p++)
		{
			line(output_image, Point(output[p][0], output[p][1]), Point(output[p][2], output[p][3]), Scalar(255, 255, 0), 1, CV_AA);
		}
	}

	////結果の表示と保存
	cv::imwrite("Line.png", result_i);
	cv::imwrite("output.png", output_image);
}




/****** 2020.11.17 n_otsuka@kinsoku.net。 updateImage は廃止する
void updateImages(boolean first)
{

	if (first)
	{
		// マスク画像を表示
		cv::imwrite("SegMask.png", mask_image);
		checkDuplicate(mask_image, labels);
		FitLines(mask_image, first);
	}
	else
	{
		cv::imwrite("SkinMask.png", mask_image);
		checkDuplicate(mask_image, labels);
		FitLines(mask_image, first);
	}




	// マスクを基に入力画像をフィルタリング
	//output_image.
	output_image.setTo(Scalar(0, 0, 0));
	input_image_rgb.copyTo(output_image, mask_image);


	if (first) {
	//細線化
	thinner(mask_image, thin_image);
	imwrite("seg_thin.png", thin_image);

	//線の検出と選択
	detectionLine();
	}
	else {
	//細線化
	thinner(mask_image, thin_image);
	imwrite("skin_thin.png", thin_image);

	//線の検出と選択
	detectionLine();
	}




	if (first)
	{
		saveParameter(first);
	}
	else
	{
		saveParameter(first);
		std::vector<Vec4i> result = pickupLine(topLine, underLine);
		if (result.size() == 0) {
			Calculation(0);
		} else {
			int diff = Distance(result, PCross);
			Calculation(diff);
			circle(result_i, PCross, 2, Scalar(0, 0, 255), -1);
			circle(result_i, Point(resultP[0][0], resultP[0][1]), 2, Scalar(255, 0, 0), -1);
			cv::imwrite("result.png", result_i);
		}
		waitKey(0);
	}

}
*/
void onTrackbar(int value, void* lptr)
{
//	createMask();
//	updateImages(false);
}

void Writing(String filename, double result) {

	if (error_F == true)
	{
		std::ofstream ofs("result.csv", std::ios::app);

		ofs << "-1" << "," << day << "," << cameranuber << "," << "NULL" << "," << position << "," << filename << endl;

		ofs.close();
	}
	else {
		std::ofstream ofs("result.csv", std::ios::app);

		ofs << "-1" << "," << day << "," << cameranuber << "," << result << "," << position << "," << filename << endl;

		ofs.close();
	}


}

/** Calculation
* @brief ピクセル単位の距離を実寸(mm 単位)に換算してファイルに追記する
*
*/
void Calculation(int diff)
{

	//num1:一ピクセルあたりの長さ(㎜)
	double num1, result = 0, result2 = 0;
	//int num2, num3, *param1, *param2 = 0;
	Point pos1;
	String FileName;
	int path = inputFile.find_last_of("\\") + 1;

	std::string str;

	FileName = inputFile.substr(path);

	num1 = floor(100 * (length / Pixel)) / 100;

	result = diff * num1;
	result2 = diff * Label_pix;

	printf("クリアランス距離 = %f\n", result2);

	Writing(FileName, result2);

}



void thinner(Mat src, Mat dst) {
	cv::Mat *kpb = new cv::Mat[8];
	cv::Mat *kpw = new cv::Mat[8];
	kpb[0] = (cv::Mat_<float>(3, 3) << 1, 1, 0, 1, 0, 0, 0, 0, 0);
	kpb[1] = (cv::Mat_<float>(3, 3) << 1, 1, 1, 0, 0, 0, 0, 0, 0);
	kpb[2] = (cv::Mat_<float>(3, 3) << 0, 1, 1, 0, 0, 1, 0, 0, 0);
	kpb[3] = (cv::Mat_<float>(3, 3) << 0, 0, 1, 0, 0, 1, 0, 0, 1);
	kpb[4] = (cv::Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 1, 0, 1, 1);
	kpb[5] = (cv::Mat_<float>(3, 3) << 0, 0, 0, 0, 0, 0, 1, 1, 1);
	kpb[6] = (cv::Mat_<float>(3, 3) << 0, 0, 0, 1, 0, 0, 1, 1, 0);
	kpb[7] = (cv::Mat_<float>(3, 3) << 1, 0, 0, 1, 0, 0, 1, 0, 0);

	kpw[0] = (cv::Mat_<float>(3, 3) << 0, 0, 0, 0, 1, 1, 0, 1, 0);
	kpw[1] = (cv::Mat_<float>(3, 3) << 0, 0, 0, 0, 1, 0, 1, 1, 0);
	kpw[2] = (cv::Mat_<float>(3, 3) << 0, 0, 0, 1, 1, 0, 0, 1, 0);
	kpw[3] = (cv::Mat_<float>(3, 3) << 1, 0, 0, 1, 1, 0, 0, 0, 0);
	kpw[4] = (cv::Mat_<float>(3, 3) << 0, 1, 0, 1, 1, 0, 0, 0, 0);
	kpw[5] = (cv::Mat_<float>(3, 3) << 0, 1, 1, 0, 1, 0, 0, 0, 0);
	kpw[6] = (cv::Mat_<float>(3, 3) << 0, 1, 0, 0, 1, 1, 0, 0, 0);
	kpw[7] = (cv::Mat_<float>(3, 3) << 0, 0, 0, 0, 1, 1, 0, 0, 1);

	cv::Mat src_w(src.rows, src.cols, CV_32FC1);
	cv::Mat src_b(src.rows, src.cols, CV_32FC1);
	cv::Mat src_f(src.rows, src.cols, CV_32FC1);
	src.convertTo(src_f, CV_32FC1);
	src_f.mul(1. / 255.);
	cv::threshold(src_f, src_f, 0.5, 1.0, CV_THRESH_BINARY);
	cv::threshold(src_f, src_w, 0.5, 1.0, CV_THRESH_BINARY);
	cv::threshold(src_f, src_b, 0.5, 1.0, CV_THRESH_BINARY_INV);

	double sum = 1;
	while (sum > 0) {
		sum = 0;
		for (int i = 0; i < 8; i++) {
			cv::filter2D(src_w, src_w, CV_32FC1, kpw[i]);
			cv::filter2D(src_b, src_b, CV_32FC1, kpb[i]);
			cv::threshold(src_w, src_w, 2.99, 1.0, CV_THRESH_BINARY);
			cv::threshold(src_b, src_b, 2.99, 1.0, CV_THRESH_BINARY);
			cv::bitwise_and(src_w, src_b, src_w);
			sum += cv::sum(src_w).val[0];
			cv::bitwise_xor(src_f, src_w, src_f);
			src_f.copyTo(src_w);
			cv::threshold(src_f, src_b, 0.5, 1.0, CV_THRESH_BINARY_INV);
		}
	}
	src_f.mul(255.);
	printf("src_f dims:%d, depth:%d, channels:%d\n", src_f.dims, src_f.depth(), src_f.channels());
	src_f.convertTo(thin_image, CV_8U, 255);
	printf("thin_image#2 dims:%d, depth:%d, channels:%d\n", thin_image.dims, thin_image.depth(), thin_image.channels());
}



// メイン関数
int main(int argc, char** argv)
{
	if (argc < 2) {
		usage();
		exit(0);
	}

	// 入力画像名(ファイルパス)
	string input_filename, input_filename2, input_filter;
	input_filename = string(argv[1]);
	inputFile = input_filename;


	//色を抽出する座標の取得
	Point point_Sg, point_Sp, point_m;
	int mark_r, mark_g, mark_b, filter;

	input_filter = string(argv[2]);
	inputFilter = input_filter;

	mark_r = atoi(argv[3]);
	mark_g = atoi(argv[4]);
	mark_b = atoi(argv[5]);

	day = string(argv[6]);

	position = string(argv[7]);

	cameranuber = string(argv[8]);

	input_filename2 = string(argv[9]);
	inputFile2 = input_filename2;

	filter = atoi(argv[10]);
	filtersize = filter;
	

	if (string(argv[11]) == "false") {
		switching = false;
	}
	else {
		switching = true;
	}

	// パラメーターファイル読み込み
	loadParameter();

	//テープの色探索
	mark_check(mark_r, mark_g, mark_b);
	HSVmask();


	// 画像を3チャンネル(BGR)で読み込む
	input_image_rgb = imread(input_filename, CV_LOAD_IMAGE_COLOR);
	if (input_image_rgb.empty()) {
		cerr << "入力画像が見つかりません" << endl;
		return -1;
	}

	laserOFF_image = imread(input_filename2, CV_LOAD_IMAGE_COLOR);
	if (laserOFF_image.empty()) {
		cerr << "画像が見つかりません" << endl;
		return -1;
	}

	//namedWindow("control", WINDOW_NORMAL);
	//createTrackbar("sliderHMin", "control", &h_min, 255, onTrackbar, NULL);
	//createTrackbar("sliderHMax", "control", &h_max, 255, onTrackbar, NULL);
	//createTrackbar("sliderSMin", "control", &s_min, 255, onTrackbar, NULL);
	//createTrackbar("sliderSMax", "control", &s_max, 255, onTrackbar, NULL);
	//createTrackbar("sliderVMin", "control", &v_min, 255, onTrackbar, NULL);
	//createTrackbar("sliderVMax", "control", &v_max, 255, onTrackbar, NULL);
	//createTrackbar("sliderMinLength", "control", &line_length_min, 255, onTrackbar, NULL);
	//imshow("control", NULL);

	input_image_rgb.copyTo(result_i);
	laserOFF_image.copyTo(result_l);

	// BGRからHSVへ変換
	cvtColor(input_image_rgb, hsv_image, CV_BGR2HSV, 3);
	cvtColor(laserOFF_image, laser_image_hsv, CV_BGR2HSV, 3);

	// HSV変換した画像を表示して確認
	cv::imwrite("hsv.png", hsv_image);

	//差分(減算合成)
	addWeighted(result_i, 1.0, result_l, -1.0, 0.0, laser_image);
	cv::imwrite("LaserImage.png", laser_image);
	cvtColor(laser_image, laser_image_hsv, CV_BGR2HSV, 3);
	printf("laser_image_hsv dims:%d, depth:%d, channels:%d\n", laser_image_hsv.dims, laser_image_hsv.depth(), laser_image_hsv.channels());
	printf("laser_image_hsv(900, 640)=%d, %d, %d\n", laser_image_hsv.at<Vec3b>(640, 900)[0], laser_image_hsv.at<Vec3b>(640, 900)[1], laser_image_hsv.at<Vec3b>(640, 900)[2]);

	/***********************************************/
	/* pass 1 画像全体をまとめて解析               */
	/***********************************************/

	//セグメント側
	mask_image = LineMaskForDrawAxis(laser_image_hsv, g_filter, 100, false);
	cv::imwrite("SegMask.png", mask_image);
	checkDuplicate(mask_image, labels);
	FitLines(mask_image, contoursP1, contoursP2, topResult1, topResult2, true, input_image_rgb);

	// マスクを基に入力画像をフィルタリング
	output_image.setTo(Scalar(0, 0, 0));
	input_image_rgb.copyTo(output_image, mask_image);
	saveParameter(true);

	////スキンプレート側
	mask_image = createMask(laser_image_hsv, g_filter, false);
	cv::imwrite("SkinMask.png", mask_image);
	checkDuplicate(mask_image, labels);
	FitLines(mask_image, contoursP1, contoursP2, topResult1, topResult2, false, input_image_rgb);
	
	output_image.setTo(Scalar(0, 0, 0));
	input_image_rgb.copyTo(output_image, mask_image);
	saveParameter(false);

	std::vector<Vec4i> result = pickupLine(topLine, underLine);
	if (result.size() == 0) {
		Calculation(0);
	}
	else {
		int diff = Distance(result, PCross, PSegmentEdge, labels);
		Calculation(diff);
	}

	/***********************************************/
	/* pass 2  画像をセグメント側とスキンプレート側に分けて別々に解析                                     */
	/***********************************************/

	//スキンプレート側
	Mat laser_image_hsv_skin;
	laser_image_hsv.copyTo(laser_image_hsv_skin);
	rectangle(laser_image_hsv_skin, Rect(PCross.x, 0, laser_image_hsv_skin.cols, laser_image_hsv_skin.rows), cv::Scalar(0, 0, 0), -1);
	imwrite("laser_image_hsv_skin.png", laser_image_hsv_skin);
	mask_image = createMask(laser_image_hsv_skin, g_filter, false);
	imwrite("SkinMask2.png", mask_image);
//	checkDuplicate(mask_image, labels);
	FitLines2(mask_image, contoursP1, contoursP2, topResult1, topResult2, false, input_image_rgb);
	
	output_image.setTo(Scalar(0, 0, 0));
	input_image_rgb.copyTo(output_image, mask_image);
	saveParameter(false);

	//セグメント側
	Mat laser_image_hsv_segment;

	laser_image_hsv.copyTo(laser_image_hsv_segment);
	rectangle(laser_image_hsv_segment, cv::Rect(0, 0, PCross.x, laser_image_hsv_segment.rows), cv::Scalar(0, 0, 0), -1);
	double www = SPREADER_SIZE / Label_value; // [pixel]
	rectangle(laser_image_hsv_segment, cv::Rect(PCross.x + www, 0, laser_image_hsv_segment.cols, laser_image_hsv_segment.rows), cv::Scalar(0, 0, 0), -1);

	// スキンプレート側のレーザー反射光を黒直線で塗りつぶしておく。（太さはテキトー）
	line(laser_image_hsv_segment, contoursP1, contoursP2, Scalar(0, 0, 0), 20, CV_AA);
	imwrite("laser_image_hsv_segment.png", laser_image_hsv_segment);

	mask_image = LineMaskForDrawAxis(laser_image_hsv_segment, g_filter, www, false);
	FitLines2(mask_image, contoursP1, contoursP2, topResult1, topResult2, true, input_image_rgb);
	imwrite("SegMask2.png", mask_image);

	labels.clear();
	mask_image = LineMaskForDetectSegmentEdge(laser_image_hsv_segment, g_filter, www, false);
	cv::imwrite("checkDuplicateResult.png", mask_image);
	checkDuplicate(mask_image, labels);

	Mat labelsForSegement;
	input_image_rgb.copyTo(labelsForSegement);
	for (int ii = 0; ii < labels.size(); ii++) {
		rectangle(labelsForSegement, Rect(labels[ii][0], labels[ii][1], (labels[ii][2] - labels[ii][0]), (labels[ii][3] - labels[ii][1])), cv::Scalar(0, 255, 255), 1);
	}
	imwrite("SegMask3.png", labelsForSegement);

	output_image.setTo(Scalar(0, 0, 0));
	input_image_rgb.copyTo(output_image, mask_image);
	saveParameter(true);

	result = pickupLine(topLine, underLine);
	if (result.size() == 0) {
		Calculation(0);
	}
	else {
		int diff = Distance(result, PCross, PSegmentEdge, labels);
		Calculation(diff);

		input_image_rgb.copyTo(result_i);
		for (int ii = 0; ii < labels.size(); ii++) {
			cv::rectangle(result_i, Rect(labels[ii][0], labels[ii][1], (labels[ii][2] - labels[ii][0]), (labels[ii][3] - labels[ii][1])), cv::Scalar(0, 255, 255), 1);
		}
		cv::line(result_i, topResult1, topResult2, Scalar(255, 255, 0), 1, CV_AA);
		cv::line(result_i, contoursP1, contoursP2, Scalar(255, 255, 0), 1, CV_AA);
		cv::circle(result_i, PCross, 3, cv::Scalar(0, 0, 255), -1);
		cv::circle(result_i, PSegmentEdge, 3, cv::Scalar(0, 0, 255), -1);
		cv::imwrite("result.png", result_i);

//		cv::imwrite("output.png", output_image);
	}

	return 0;
}

double Labeling(Mat mask) {
	int Label_num = NULL;
	int max = INT_MIN;
	std::array<int, 100000> number;


	//ラべリング処理
	int nLab = cv::connectedComponentsWithStats(mask, Label_LabelImg, Label_stats, Label_centroids);

	// ラベリング結果の描画色を決定
	std::vector<cv::Vec3b> colors(nLab);
	colors[0] = cv::Vec3b(0, 0, 0);
	for (int i = 1; i < nLab; ++i) {
		colors[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
	}

	// ラベリング結果の描画
	cv::Mat Dst(mask.size(), CV_8UC3);
	for (int i = 0; i < Dst.rows; ++i) {
		int *lb = Label_LabelImg.ptr<int>(i);
		cv::Vec3b *pix = Dst.ptr<cv::Vec3b>(i);
		for (int j = 0; j < Dst.cols; ++j) {
			pix[j] = colors[lb[j]];
		}
	}

	//ROIの設定(対象領域)
	for (int i = 1; i < nLab; ++i) {
		int *param = Label_stats.ptr<int>(i);

		int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
		mark_X = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		
		cv::rectangle(Dst, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 2);
	}

	//重心の出力
	for (int i = 1; i < nLab; ++i) {
		double *param = Label_centroids.ptr<double>(i);
		int x = static_cast<int>(param[0]);
		int y = static_cast<int>(param[1]);

		cv::circle(Dst, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
	}

	//面積値の出力・垂直方向の出力
	for (int i = 1; i < nLab; ++i) {
		int *param = Label_stats.ptr<int>(i);

		//総面積(ピクセル単位)100以下は無視する
		if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA] > 150) {
			number[i] = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];

			std::cout << "水平方向 : " << i << " = " << param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH] << std::endl;
			std::cout << "垂直方向 : " << i << " = " << param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT] << std::endl;
			//std::cout << "総面積 : " << i << " = " << param[cv::ConnectedComponentsTypes::CC_STAT_AREA] << std::endl;

			//ROIの左上に番号を書き込む
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
			}
		}

	}
	if (Label_num == NULL) {
		cerr << "マーキングテープが見つかりませんでした。" << endl;
		return -1;
	}



	cv::imwrite("labeling.png", Dst);
	printf("labeling.pngを出力しました。\n");

	//CorrectionValuはここに移動
	printf("Value開始\n");

	int *param = 0;
	int MAX = 0;
	int pixcel[5];

	param = Label_stats.ptr<int>(Label_num);
	pixcel[0] = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
	pixcel[1] = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
	mark_X = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
	center_X = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH] / 2;

	if (pixcel[0] < pixcel[1]) {
		MAX = pixcel[1];
	}
	else if (pixcel[0] > pixcel[1]) {
		MAX = pixcel[0];
	}
	else {
		MAX = (pixcel[0] + pixcel[1]) / 2;
	}


	Label_value = floor(100 * (Label_length / MAX)) / 100;
	printf("ラベルの大きさ = %d [pixel]\n", MAX);
	printf("補正値 = %f\n", Label_value);

	return Label_value;
}

//HSVによる色抽出
void HSVmask() {

	//入力画像
	String input_filename = inputFile;

	//画像を3チャンネル(BGR)で読み込む
	Mat input_image_rgb = imread(input_filename, CV_LOAD_IMAGE_COLOR);
	if (input_image_rgb.empty()) {
		cerr << "入力画像が見つかりません。" << endl;
		return;
	}

	//BGRからHSVへ変換
	Mat hsv_image, mask_image, output_image;
	Mat image[5];
	Mat mask[5];
	Mat green;
	cvtColor(input_image_rgb, hsv_image, CV_BGR2HSV, 3);

	//inRangeを用いてフィルタリング
	inRange(hsv_image, cv::Scalar(30, MarkS_MIN, MarkV_MIN), cv::Scalar(MarkH_MAX, MarkS_MAX, MarkV_MAX), image[0]);
	inRange(hsv_image, cv::Scalar(MarkH_MIN, MarkS_MIN, MarkV_MIN), cv::Scalar(85, MarkS_MAX, MarkV_MAX), image[1]);

	inRange(hsv_image, cv::Scalar(30, 100, 100), cv::Scalar(85, 255, 255), image[2]);
	inRange(hsv_image, cv::Scalar(30, 85, 100), cv::Scalar(85, 255, 255), image[3]);

	addWeighted(image[0], 1.0, image[1], 1.0, 0.0, mask[0]);//合体
	addWeighted(image[2], 1.0, image[3], 1.0, 0.0, mask[1]);//合体
	addWeighted(mask[0], 1.0, mask[1], 1.0, 0.0, mask_image);//上のやつと合体

	//マスク画像
	cv::imwrite("Label_mask.png", mask_image);

	//マスクをもとに入力画像をフィルタリング
	input_image_rgb.copyTo(output_image, mask_image);

	//結果の保存
	cv::imwrite("Label_output.png", output_image);

	double mask_image_pix = 0;

	mask_image_pix = Labeling(mask_image);

	Label_pix = mask_image_pix;


	return;
}

//マークのHSVを調べる
void mark_check(int r, int g, int b) {
	//HSVパラメーター
	double sikisou, saido, meido;

	cv::Mat srcimg = cv::imread(inputFile);
	cv::Mat3b dotimg = srcimg;

	double R = r;
	double G = g;
	double B = b;

	double MAX = max(max(R, G), B);
	double MIN = min(min(R, G), B);
	meido = MAX / 256 * 255;

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
		saido = (MAX - MIN) / MAX * 255;
	}
	printf("マーキング	色相[%.2f] 彩度[%.2f] 明度[%.2f]\n", sikisou / 2, saido, meido);

	MarkH_MAX = sikisou / 2;
	MarkS_MIN = saido;
	MarkV_MIN = meido;

	return;
}