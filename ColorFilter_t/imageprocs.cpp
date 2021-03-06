﻿#include "stdafx.h"
#include "imageprocs.h"


// findContours に期待する contour の数
const int MAX_NO_OF_CONTOUR = 200;	// <-- これ以上だと多過ぎ
const int MIN_NO_OF_CONTOUR = 50;	// <-- これ以下だと少な過ぎ
const int MINIMAL_CONTOUR_SIZE = 40;	// この値以下の contour はノイズと見做して無視する

color_filter g_filter;
Point reserve1, reserve2, topReserve1, topReserve2;

										
/** FitLines
*
*
*
*/
/** FitLines
*
*
*
*/
void FitLines(Mat fit, Point& contoursP1, Point& contoursP2, Point& topResult1, Point& topResult2, boolean first, Mat& input_image_rgb) {
	vector<Vec4i> hierarchy;
	Mat contours_img;

	input_image_rgb.copyTo(contours_img);

	//1番大きい奴と二番目  
	double max_area = 0., min_area = 0.;
	int max_area_contour = -1, min_area_cotour = -1;

	vector<vector<Point>> contours;

	findContours(fit, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	cout << "no of contours =" << contours.size() << std::endl;

	vector<vector<Point> > contours_subset;
	int max_size = 0;
	for (int j = 0; j < contours.size(); j++) {
		int contourSize = contours.at(j).size();
		if (contourSize >= MINIMAL_CONTOUR_SIZE) {
			contours_subset.push_back(contours.at(j));
			if (contourSize > max_size) {
				max_size = contourSize;
			}
		}
	}
	cout << "size of contours_subset=" << contours_subset.size() << std::endl;
	cout << "max contour_subset.size=" << max_size << std::endl;

	for (int j = 0; j < contours_subset.size(); j++) {
		int count = contours_subset.at(j).size();
		double x = 0;
		double y = 0;
		for (int k = 0; k < count; k++) {
			x += contours_subset.at(j).at(k).x;
			y += contours_subset.at(j).at(k).y;
		}
		x /= count;
		y /= count;
		cout << "contour_subset[" << j << "].x=" << x << " y=" << y << std::endl;
		circle(contours_img, Point(x, y), 50, Scalar(0, 0, 255), 3, 4);
		drawContours(contours_img, contours_subset, -1, Scalar(0, 255, 0), 1);

		Vec4f fitlines;
		double inclination;
		int incX, incY;
		int coun = 0;
		vector<Vec4i> conReslt;
		Point startPoint, endPoint;

		/** @brief contour に含まれる各点から該直線までの「距離の二乗」の和が最小になるような直線を求める
		@parmeter
		contour 点群
		fitlines Vec4f(vx, vy, x0, y0), (vx, vy) は該直線の方向を示す単位ベクトル。 (x0, y0) は直線の通過点のひとつ。
		CV_DIST_L2 各点と該直線の間の距離を二乗和が最小になるような直線を算出することを指定する
		param
		reps Sufficient accuracy for the radius (distance between the coordinate origin and the line).
		aeps Sufficient accuracy for the angle. 0.01 would be a good default value for reps and aeps.

		@ref https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#gaf849da1fdafa67ee84b1e9a23b93f91f
		*/
		fitLine(contours_subset[j], fitlines, CV_DIST_L2, 0, 0.1, 0.01);

		startPoint.x = fitlines.val[2] - fitlines.val[0] * 200.0;
		startPoint.y = fitlines.val[3] - fitlines.val[1] * 200.0;
		endPoint.x = fitlines.val[2] + fitlines.val[0] * 200.0;
		endPoint.y = fitlines.val[3] + fitlines.val[1] * 200.0;

		line(contours_img,
			startPoint,     // １つ目の線分の座標
			endPoint,     // ２つ目の線分の座標 
			cv::Scalar(0, 255, 255),   // 色 
			1,                       // 太さ 
			cv::LINE_8
		);

		inclination = atan2(fitlines.val[1], fitlines.val[0]);

		while (inclination <= -M_PI / 2) {
			inclination += M_PI;
		}
		while (inclination > M_PI / 2) {
			inclination -= M_PI;
		}

		if (inclination > (-30.0 * (M_PI / 180)) && inclination < (-20.0 * (M_PI / 180))) {
			if (reserve1.x == NULL || endPoint.x < reserve2.x)
			{
				reserve1 = startPoint;
				reserve2 = endPoint;
			}
		}
		else if (abs(inclination) < 2. * (M_PI / 180.))
		{
			if (topReserve1.x == 0 || startPoint.x < topReserve1.x) {
				topReserve1 = startPoint;
				topReserve2 = endPoint;
			}
		}
	} // foreach (contours_subset) 

	if (first == true) {
		cv::imwrite("Contours11.png", contours_img);
	}
	else {
		cv::imwrite("Contour22.png", contours_img);
	}

	contoursP1 = reserve1;
	contoursP2 = reserve2;
	topResult1 = topReserve1;
	topResult2 = topReserve2;

}

void FitLines2(Mat fit, Point& contoursP1, Point& contoursP2, Point& topResult1, Point& topResult2, boolean first, Mat& input_image_rgb) {
	vector<Vec4i> hierarchy;
	Mat contours_img;

	input_image_rgb.copyTo(contours_img);

	//1番大きい奴と二番目
	double max_area = 0., min_area = 0.;
	int max_area_contour = -1, min_area_cotour = -1;


	vector<vector<Point>> contours;

	findContours(fit, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	cout << "no of contours =" << contours.size() << std::endl;

	vector<vector<Point> > contours_subset;
	vector<Point> contours_merged;
	int max_size = 0;
	int max_contour_id = -1;
	for (int j = 0; j < contours.size(); j++) {
		int contourSize = contours.at(j).size();
		if (contourSize >= MINIMAL_CONTOUR_SIZE) {
			contours_merged.insert(contours_merged.end(), contours.at(j).begin(), contours.at(j).end());
			contours_subset.push_back(contours.at(j));
			if (contourSize > max_size) {
				max_size = contourSize;
				max_contour_id = j;
			}
		}
	}
	cout << "size of contours_subset=" << contours_subset.size() << std::endl;
	cout << "max contour_subset.size=" << max_size << std::endl;

	for (int j = 0; j < contours_subset.size(); j++) {
		int count = contours_subset.at(j).size();
		double x = 0;
		double y = 0;
		for (int k = 0; k < count; k++) {
			x += contours_subset.at(j).at(k).x;
			y += contours_subset.at(j).at(k).y;
		}
		x /= count;
		y /= count;
		cout << "contour_subset[" << j << "].x=" << x << " y=" << y << std::endl;
		circle(contours_img, Point(x, y), 50, Scalar(0, 0, 255), 3, 4);
		drawContours(contours_img, contours_subset, -1, Scalar(0, 255, 0), 1);
	} // foreach (contours_subset)

	Vec4f fitlines;
	double inclination;
	int incX, incY;
	int coun = 0;
	vector<Vec4i> conReslt;
	Point startPoint, endPoint;
	  
	/** @brief contour に含まれる各点から該直線までの「距離の二乗」の和が最小になるような直線を求める
	@parmeter
	contour 点群
	fitlines Vec4f(vx, vy, x0, y0), (vx, vy) は該直線の方向を示す単位ベクトル。 (x0, y0) は直線の通過点のひとつ。
	CV_DIST_L2 各点と該直線の間の距離を二乗和が最小になるような直線を算出することを指定する
	param
	reps Sufficient accuracy for the radius (distance between the coordinate origin and the line).
	aeps Sufficient accuracy for the angle. 0.01 would be a good default value for reps and aeps.

	@ref https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#gaf849da1fdafa67ee84b1e9a23b93f91f
	*/

	fitLine(contours.at(max_contour_id), fitlines, CV_DIST_L2, 0, 0.1, 0.002);

	startPoint.x = fitlines.val[2] - fitlines.val[0] * 1000.0;
	startPoint.y = fitlines.val[3] - fitlines.val[1] * 1000.0;
	endPoint.x = fitlines.val[2] + fitlines.val[0] * 1000.0;
	endPoint.y = fitlines.val[3] + fitlines.val[1] * 1000.0;

	line(contours_img,
		startPoint,     // １つ目の線分の座標
		endPoint,     // ２つ目の線分の座標
		cv::Scalar(0, 255, 255),   // 色
		1,                       // 太さ
		cv::LINE_8
	);

	inclination = atan2(fitlines.val[1], fitlines.val[0]);

	while (inclination <= -M_PI / 2) {
		inclination += M_PI;
	}
	while (inclination > M_PI / 2) {
		inclination -= M_PI;
	}

	if (!first && inclination > (-30.0 * (M_PI / 180)) && inclination < (-20.0 * (M_PI / 180))) {
//		if (reserve1.x == 0 || endPoint.x < reserve2.x)
//		{
			reserve1 = startPoint;
			reserve2 = endPoint;
//		}
	}
	else if (first && abs(inclination) < 2. * (M_PI / 180.))
	{
//		if (topReserve1.x == NULL || startPoint.x < topReserve1.x) {
			topReserve1 = startPoint;
			topReserve2 = endPoint;
//		}
	}

	if (first == true) {
		cv::imwrite("ContoursFisrt.png", contours_img);
	}
	else {
		cv::imwrite("ContourSecond.png", contours_img);
	}

	contoursP1 = reserve1;
	contoursP2 = reserve2;
	topResult1 = topReserve1;
	topResult2 = topReserve2;

}

										/**
* @brief labels に含まれる四角領域群の中から、指定された contour を含む四角領域を返す
* @parameter
*
*
*/
vector<Vec4i>  selectLabel(Point a, Point b, vector<Vec4i> masksss) {
	vector<Vec4i> resultP;
	//Mat Dst;
	//input_image_rgb.copyTo(Dst);

	Point c;
	c.x = (a.x + b.x) / 2;
	c.y = (a.y + b.y) / 2;

	for (; masksss.size() > 0;) {

		if (masksss[0][0] <= c.x && masksss[0][2] >= c.x) {
			if (masksss[0][1] <= c.y && masksss[0][3] >= c.y) {
				resultP.push_back(masksss[0]);

			}
		}
		masksss.erase(masksss.begin());
	}

	for (int y = 0; y < resultP.size(); y++) {

		//cv::rectangle(Dst, cv::Rect(resultP[y][0], resultP[y][1], resultP[y][2] - resultP[y][0], resultP[y][3] - resultP[y][1]), cv::Scalar(0, 255, 0), 2);

	}

	//cv::imwrite("SelectRect.png", Dst);

	int diff1, diff2 = 0;
	if (resultP.size() > 1) {

		for (resultP.size() > 2; resultP.size() > 1;) {


			diff1 = a.x - resultP[0][0];
			diff2 = a.x - resultP[1][0];

			if (diff1 >= 0)
			{
				if (diff2 >= 0)
				{
					if (diff1 < diff2)
					{
						resultP.erase(resultP.begin() + 1);
					}
					else {
						resultP.erase(resultP.begin());
					}

				}
				else {
					resultP.erase(resultP.begin() + 1);
				}

			}
			else {
				resultP.erase(resultP.begin());
			}
		}
	}
	//if (resultP.size() != 0) {
	//	diff1 = a.x - resultP[0][0];
	//	Mat LastI;
	//	input_image_rgb.copyTo(LastI);
	//	cv::rectangle(LastI, cv::Rect(resultP[0][0], resultP[0][1], resultP[0][2] - resultP[0][0], resultP[0][3] - resultP[0][1]), cv::Scalar(0, 255, 0), 1);
	//	cv::imwrite("RECT.png", LastI);
	//}
	return resultP;

}

vector<Vec4i>  selectLabelOnTheLine(Point a, Point b, vector<Vec4i> masksss) {
	vector<Vec4i> masksOnTheLine;

	int indexOfMostLeftLabel = -1;
	double leftOfMostLeftLabel = DBL_MAX;
	double k = (double)(b.y - a.y) / (b.x - a.x);
	for (int ii = 0; ii < masksss.size(); ++ii) {
		// masksss[ii] 0:左端 / 1:上端 / 2:右端 / 3:下端
		double crossPointLeft = a.y + k * masksss[ii][0];
		double crossPointRight = a.y + k * masksss[ii][2];
		if (crossPointLeft >= masksss[ii][1] && crossPointLeft <= masksss[ii][3]
			&& crossPointRight >= masksss[ii][1] && crossPointRight <= masksss[ii][3]) {
			if (masksss[ii][0] < leftOfMostLeftLabel) {
				indexOfMostLeftLabel = ii;
				leftOfMostLeftLabel = masksss[ii][0];
			}
		}
		if (indexOfMostLeftLabel >= 0 && abs(masksss[ii][0] - masksss[ii][2]) > 100) {
			masksOnTheLine.push_back(masksss[indexOfMostLeftLabel]);
		}
	}
	return masksOnTheLine;
}

//スキンプレート
Mat createMask(Mat laser_image_hsv, color_filter filter, boolean fine) {
	Mat mask_image;
	//double sigma = 0;
	//cv::Mat gaussian;
	//cv::GaussianBlur(laser_image, gaussian, cv::Size(filtersize, filtersize), sigma);
	//cv::imwrite("gaussian.png", gaussian);

	vector<Vec4i> hierarchy;
	vector<vector<Point>> contours;
	double dvmin = (filter.v_max - filter.v_min) / 2;
	double v_min_skin = (filter.v_max + filter.v_min) / 2;
	int max_no_of_contour = MAX_NO_OF_CONTOUR;
	int min_no_of_contour = MIN_NO_OF_CONTOUR;
	if (fine) {
		max_no_of_contour /= 2;
		min_no_of_contour /= 2;
	}

	//inRange(gaussian, cv::Scalar(h_max, s_min, v_min), cv::Scalar(179, s_max, v_max), upper_red_hue_range);
	//inRange(gaussian, cv::Scalar(0, s_min, v_min), cv::Scalar(h_min, s_max, v_max), lower_red_hue_range);
	while (true) {
		Mat lower_red_hue_range, upper_red_hue_range;

		if (filter.h_min <= filter.h_max) {
			inRange(laser_image_hsv, cv::Scalar(filter.h_min, filter.s_min, v_min_skin), cv::Scalar(filter.h_max, filter.s_max, filter.v_max), mask_image);
		}
		else {
			inRange(laser_image_hsv, cv::Scalar(filter.h_min, filter.s_min, v_min_skin), cv::Scalar(180., filter.s_max, filter.v_max), lower_red_hue_range);
			inRange(laser_image_hsv, cv::Scalar(0., filter.s_min, v_min_skin), cv::Scalar(filter.h_max, filter.s_max, filter.v_max), upper_red_hue_range);
			add(lower_red_hue_range, upper_red_hue_range, mask_image);
		}

		findContours(mask_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		cout << "createMask> v_min_skin=" << v_min_skin << " contours.size()=" << contours.size() << std::endl;

		if (dvmin < 1) {
			break;
		}
		else if (contours.size() > max_no_of_contour) {
			v_min_skin += dvmin;
			dvmin /= 2;
		}
		else if (contours.size() < min_no_of_contour) {
			v_min_skin -= dvmin;
			dvmin /= 2;
		}
		else {
			break;
		}


	}

	cv::imwrite("skinMask.png", mask_image);
	return mask_image;
}

//セグメント
Mat LineMaskForDrawAxis(Mat laser_image_hsv, color_filter filter, double min_contour_size, boolean fine) {
	Mat mask_image;
	double sigma = 0;
	vector<Vec4i> hierarchy;
	vector<vector<Point>> contours;
	double dvmin = (filter.v_max - filter.v_min) / 20;
	double v_min_segment = filter.v_max - dvmin;
	int max_no_of_contour = MAX_NO_OF_CONTOUR;
	int min_no_of_contour = MIN_NO_OF_CONTOUR;
	if (fine) {
		max_no_of_contour /= 2;
		min_no_of_contour /= 2;
	}

	while (true) {
		Mat lower_red_hue_range, upper_red_hue_range;

		if (filter.h_min <= filter.h_max) {
			inRange(laser_image_hsv, cv::Scalar(filter.h_min, filter.s_min, v_min_segment), cv::Scalar(filter.h_max, filter.s_max, filter.v_max), mask_image);
		}
		else {
			inRange(laser_image_hsv, cv::Scalar(filter.h_min, filter.s_min, v_min_segment), cv::Scalar(180., filter.s_max, filter.v_max), lower_red_hue_range);
			inRange(laser_image_hsv, cv::Scalar(0., filter.s_min, v_min_segment), cv::Scalar(filter.h_max, filter.s_max, filter.v_max), upper_red_hue_range);
			add(lower_red_hue_range, upper_red_hue_range, mask_image);
		}
		findContours(mask_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		cout << "LineMaskForDrawAxis> v_min_segment=" << v_min_segment << " contours.size()=" << contours.size() << std::endl;
		double largest_contouur_size = 0.;
		int largest_contour_no = -1;
		for (int ii = 0; ii < contours.size(); ii++) {
			if (contours.at(ii).size() > min_contour_size) {
				largest_contour_no = ii;
				break;
			}
		}
		if (largest_contour_no >= 0) {
			break;
		}
		else if (v_min_segment < 10) {
			break;
		}
		//else if (contours.size() > max_no_of_contour) {
		//	v_min_segment += dvmin;
		//	dvmin /= 2;
		//}
		//else if (contours.size() < min_no_of_contour) {
		//	v_min_segment -= dvmin;
		//	dvmin /= 2;
		//}
		else {
			v_min_segment -= dvmin;
		}
	}

	return mask_image;
}

Mat LineMaskForDetectSegmentEdge(Mat laser_image_hsv, color_filter filter, double min_contour_size, boolean fine) {
	Mat mask_image;
	double sigma = 0;
	vector<Vec4i> hierarchy;
	vector<vector<Point>> contours;
	double dvmin = (filter.v_max - filter.v_min) / 2;
	double v_min_segment = (filter.v_max + filter.v_min) / 2.;
	int max_no_of_contour = MAX_NO_OF_CONTOUR;
	int min_no_of_contour = MIN_NO_OF_CONTOUR;

	while (true) {
		Mat lower_red_hue_range, upper_red_hue_range;

		if (filter.h_min <= filter.h_max) {
			inRange(laser_image_hsv, cv::Scalar(filter.h_min, filter.s_min, v_min_segment), cv::Scalar(filter.h_max, filter.s_max, filter.v_max), mask_image);
		}
		else {
			inRange(laser_image_hsv, cv::Scalar(filter.h_min, filter.s_min, v_min_segment), cv::Scalar(180., filter.s_max, filter.v_max), lower_red_hue_range);
			inRange(laser_image_hsv, cv::Scalar(0., filter.s_min, v_min_segment), cv::Scalar(filter.h_max, filter.s_max, filter.v_max), upper_red_hue_range);
			add(lower_red_hue_range, upper_red_hue_range, mask_image);
		}
		findContours(mask_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		cout << "LineMaskForDetectSegmentEdge> v_min_segment=" << v_min_segment << " contours.size()=" << contours.size() << std::endl;

		if (dvmin < 0) {
			break;
		}
		else if (contours.size() > max_no_of_contour) {
			v_min_segment += dvmin;
			dvmin /= 2;
		}
		else if (contours.size() < min_no_of_contour) {
			v_min_segment -= dvmin;
			dvmin /= 2;
		}
		else {
			break;
		}
	}
	return mask_image;
}


/* @brief スキンプレート側線分とセグメント側線分の交点を計算し、交点とセグメント側線分の左端の隔たりを返す
*  @parameter
*    a スキンプレート側線分とセグメント側線分の座標データ
*	 PCross 交点
*/
int Distance(vector<Vec4i> a, Point& PCross, Point& PSegmentEdge, vector<Vec4i> mask)
{

	//int num, *param1, *param2 = 0;
	double diff = 0;

	Point P1, P2, P3, P4, tmp;

	P1.x = a[0][2];
	P1.y = a[0][3];
	P2.x = a[0][0];
	P2.y = a[0][1];
	P3.x = a[1][2];
	P3.y = a[1][3];
	P4.x = a[1][0];
	P4.y = a[1][1];


	//配置チェック
	if (P1.x < P2.x) {

		tmp = P1;
		P1 = P2;
		P2 = tmp;
	}
	if (P3.x < P4.x)
	{
		tmp = P3;
		P3 = P4;
		P4 = tmp;
	}

	double ksi, eta, delta;
	double ramda, mu;

	ksi = (P4.y - P3.y)*(P4.x - P1.x) - (P4.x - P3.x)*(P4.y - P1.y);

	eta = (P2.x - P1.x)*(P4.y - P1.y) - (P2.y - P1.y)*(P4.x - P1.x);

	delta = (P2.x - P1.x)*(P4.y - P3.y) - (P2.y - P1.y)*(P4.x - P3.x);


	ramda = ksi / delta;

	mu = eta / delta;


	PCross.x = P1.x + ramda * (P2.x - P1.x);
	PCross.y = P1.y + ramda * (P2.y - P1.y);
	PSegmentEdge.x = 0.;
	PSegmentEdge.y = 0.;

	vector<Vec4i> resultP = selectLabelOnTheLine(P2, P1, mask);
	if (resultP.size() > 0) {
		diff = resultP[0][0] - PCross.x;
		PSegmentEdge.x = resultP[0][0];
		PSegmentEdge.y = resultP[0][1];
	}
	else {
		printf("RECTの作成に失敗しました\n");
		diff = P2.x - PCross.x;
	}


	printf("交点X：%d\n交点Y:%d\n", PCross.x, PCross.y);
	printf("差:%f\n", diff);
	return diff;

}

/**
*  @brief	入力2値画像をラベリング
*
*
*
*/
void checkDuplicate(Mat input_image, vector<Vec4i>& labels) {
	Mat sg_image, sg_stats, sg_centroids;

	std::array<int, 100000> number;
	int x, y, height, width;

	int nLab = cv::connectedComponentsWithStats(input_image, sg_image, sg_stats, sg_centroids);
	cv::imwrite("sg_Lablig.png", sg_image);

	std::vector<cv::Vec3b> colors(nLab);
	colors[0] = cv::Vec3b(0, 0, 0);
	for (int i = 1; i < nLab; ++i) {
		colors[i] = cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
	}

	// ラベリング結果の描画
	cv::Mat Dst(input_image.size(), CV_8UC3);
	for (int i = 0; i < Dst.rows; ++i) {
		int *lb = sg_image.ptr<int>(i);
		cv::Vec3b *pix = Dst.ptr<cv::Vec3b>(i);
		for (int j = 0; j < Dst.cols; ++j) {
			pix[j] = colors[lb[j]];
		}
	}

	//ROIの設定
	for (int i = 1; i < nLab; ++i) {
		int *param = sg_stats.ptr<int>(i);

		x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
		y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
		height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
		width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

		labels.push_back({ x,y,x + width,y + height });
		//cv::rectangle(Dst, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 2);
	}
	cv::imwrite("sg_Lablig.png", Dst);
}

