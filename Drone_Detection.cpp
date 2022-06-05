#include "stdafx.h"
#include <regex>
#include <algorithm>
#include <iterator>
#include <cfloat>
#include <cuchar>
#include <ctime>
#include <time.h>
#include <process.h>
#include <conio.h>
#include <atlbase.h>
#include <atlcom.h>
#include <atlstr.h>
#include <CommCtrl.h>
#include <math.h>
#include <objbase.h>
#include <vector>
#include <thread>
#include <iostream>
#include <stdio.h>
#include "Resource.h"
#include <wchar.h>
#include <tchar.h>
#include <commdlg.h>
#include <errno.h>
#include <chrono>
#include <winuser.h>
#include "zmouse.h"
#include <Windows.h>
#include "Drone_Detection.h"
#include <opencv2\opencv.hpp>
#include "opencv2/core.hpp"
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/objdetect.hpp"
#include <opencv2\objdetect\objdetect.hpp>
#include "opencv2\objdetect\detection_based_tracker.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/features2d/hal/interface.h>
#include <sstream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <ctime>
#include <sys/stat.h>
#include <dirent.h>
#include <sys/types.h>
#include <functional>
#include <filesystem>
#include <mmsystem.h>
using namespace std;
using namespace cv;
#define READ_TIMEOUT 5000
#define MAX_LOADSTRING 100
#define TARGET_NONE 0
#define TARGET_RED 1
#define TARGET_GREEN 2
#define second_Threshould 255
#define first_Threshould 72
#define first_Threshould_Per 20
#define diff1 36
#define diff1_Per 25
#define N_cloud 8
#define N_negative 40
#define N_sky 20
#define N_mountain 20
#define C_sky 20
#define C_mountain 20
#define C_r1 3
#define C_dx 17
#define C_dy 8
#define IDC_CAPTURE_BTN (56)
#define IDC_FEATURE_SAVE_BTN (57)
uchar **arr;
Mat frame;
Mat frame_GRAY;
Mat cloud_Mask;
Mat prev_cloud_Mask;
Mat fixed_Mask;
Mat mountain_Mask;
Mat bad_pixel_Mask;
Mat bin_Mask;
Mat prev_In_Current;
struct target
{
	Point p;
	bool Is_isolated;
	int temp_type;
	int type;
	int prev_index;
	double speed_x;
	double speed_y;
	bool bad_pixel;
	bool pervIS_target_in_Current;
	double target_print;
};
vector<target> current_targets;
vector<target> previous_targets;
string name_cloud;
string name_source;
string name_mountain;
string name_fixed;
string name_bad_pixel;
string name_bin;
string name_prev_Incurrent;
string name_result;
int frame_index = -1;
int cv_x = 85;
int cv_y = 0;
bool c_First_Time = true;
HWND feature_Save_btn;
HWND capture_Img_btn;
HINSTANCE hInst;
WCHAR szTitle[MAX_LOADSTRING];
WCHAR szWindowClass[MAX_LOADSTRING];
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);
void capture_Video(string video_path);
void frame_2_point(Mat frame_GBR);
void cloud(int height, int width);
bool in_First(int i, int j, int i2, int j2);
void fixed_target(int height, int width);
void mountain(int height, int width);
void fill_target(int height, int width);
void target_Add(int x, int y, int temp_type, int type);
void bad_pixel(int height, int width);
void remove_bad_pixel();
void scan_prev_in_current(int height, int width);
void initilize_Targets_lists();
void calculate_target_Print(int i);
void temporary_decision();
void final_decision();
void draw_results();
void current_2_previous();

void capture_Video(string video_path)
{
	VideoCapture capture(video_path);
	//VideoCapture capture("http://192.168.137.136:8080/video");
	if (!capture.isOpened())
		throw "Error when reading steam_avi";

	for (;;)
	{
		capture >> frame;
		if (frame.size() == cv::Size(0,0))
		{
			destroyAllWindows();
			break;
		}
		/*Rect rectan;
		rectan = Rect(50, 35, 870, 480);
		frame = frame(rectan);*/
		cv::namedWindow("source", 1);
		cv::moveWindow("source", 500, 80);
		imshow("source", frame);
		frame_2_point(frame);
		waitKey(1);
	}
	return;
}

void frame_2_point(Mat frame_GBR)
{
	cvtColor(frame_GBR, frame_GRAY, COLOR_BGR2GRAY);
	frame_index++;
	name_source = "source_" + std::to_string(frame_index) + ".png";
	const wchar_t* dir;
	dir = L"F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\source\\";
	if (_wmkdir(dir) != -1 && c_First_Time)
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\source\\" + name_source, frame_GRAY);
	}
	else
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\source\\" + name_source, frame_GRAY);
	}
	int height = frame_GRAY.rows;
	int width = frame_GRAY.cols;
	arr = (uchar **)malloc(height * sizeof(uchar *));
	for (int i = 0; i < height; i++)
	{
		uchar* row = frame_GRAY.ptr<uchar>(i);
		arr[i] = (uchar *)malloc(width * sizeof(uchar));
		arr[i] = row;
	}

	cloud(height, width);
	fixed_target(height, width);
	mountain(height, width);
	fill_target(height, width);
	bad_pixel(height, width);
	remove_bad_pixel();
	scan_prev_in_current(height, width);
	initilize_Targets_lists();
	temporary_decision();
	final_decision();
	draw_results();
	current_2_previous();
	c_First_Time = false;
}

void cloud(int height, int width)
{
	cloud_Mask = Mat(height, width, 0);
	cloud_Mask = 0;
	int Nn = 0;
	int ndiff1 = 0;
	int ndiff2 = 0;
	int ndiff3 = 0;
	for (int i = 8; i < height - 8; i++)
	{
		uchar* cloud = cloud_Mask.ptr<uchar>(i);
		for (int j = 8; j < width - 8; j++)
		{
			if ((int)arr[i][j] >= first_Threshould && (int)arr[i][j] <= second_Threshould)
			{
				for (int i1 = i - 1; i1 <= i + 1; i1++)
				{
					for (int j1 = j - 1; j1 <= j + 1; j1++)
					{
						if ((int)arr[i1][j1] >(int)arr[i][j])
						{
							Nn++;
						}
						if (((int)arr[i1][j1] < (int)arr[i][j] + diff1 / 2) && ((int)arr[i1][j1] > (int)arr[i][j] - diff1 / 2))
						{
							if ((i1 == i) && (j1 == j)) continue;
							ndiff1++;
						}
					}
				}
				for (int i2 = i - 2; i2 <= i + 2; i2++)
				{
					for (int j2 = j - 2; j2 <= j + 2; j2++)
					{
						if ((int)arr[i2][j2] > (int)arr[i][j])
						{
							Nn++;
						}
						if (((int)arr[i2][j2] < (int)arr[i][j] + diff1) && ((int)arr[i2][j2] > (int)arr[i][j] - diff1))
						{
							if (in_First(i, j, i2, j2)) continue;
							ndiff2++;
						}
					}
					if ((ndiff1 + ndiff2) > N_cloud || Nn > N_negative)
					{
						cloud[j] = 0;
						break;
					}
				}
				ndiff3 = ndiff1 + ndiff2;
				if (ndiff3 <= N_cloud && Nn <= N_negative)
				{
					cloud[j] = 255;
				}
				else
				{
					cloud[j] = 0;
				}
			}
			else
			{
				cloud[j] = 0;
			}
			ndiff1 = 0;
			ndiff2 = 0;
			ndiff3 = 0;
			Nn = 0;
		}
	}
	name_cloud = "cloud_Mask_" + std::to_string(frame_index) + ".png";
	cv::namedWindow("cloud", 1);
	cv::moveWindow("cloud", 20, 400);
	cv::imshow("cloud", cloud_Mask);
	const wchar_t* dir;
	dir = L"F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\cloud\\";
	if (_wmkdir(dir) != -1 && c_First_Time)
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\cloud\\" + name_cloud, cloud_Mask);
	}
	else
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\cloud\\" + name_cloud, cloud_Mask);
	}
}

bool in_First(int i, int j, int i2, int j2)
{
	bool realy_in = false;
	if ((i2 == i - 1) && (j2 == j - 1)) realy_in = true;
	if ((i2 == i - 1) && (j2 == j)) realy_in = true;
	if ((i2 == i - 1) && (j2 == j + 1)) realy_in = true;
	if ((i2 == i) && (j2 == j - 1)) realy_in = true;
	if ((i2 == i) && (j2 == j)) realy_in = true;
	if ((i2 == i) && (j2 == j + 1)) realy_in = true;
	if ((i2 == i + 1) && (j2 == j - 1)) realy_in = true;
	if ((i2 == i + 1) && (j2 == j)) realy_in = true;
	if ((i2 == i + 1) && (j2 == j + 1)) realy_in = true;
	return realy_in;
}

void fixed_target(int height, int width)
{
	fixed_Mask = Mat(height, width, 0);
	fixed_Mask = 255;
	if (!c_First_Time)
	{
		for (int i = 8; i < height - 8; i++)
		{
			uchar* fixed_pixel = fixed_Mask.ptr<uchar>(i);
			for (int j = 8; j < width - 8; j++)
			{
				if (cloud_Mask.at<uchar>(i, j) > 0)
				{
					for (int i1 = i - C_r1; i1 <= i + C_r1; i1++)
					{
						for (int j1 = j - C_r1; j1 <= j + C_r1; j1++)
						{
							if (prev_cloud_Mask.at<uchar>(i1, j1) > 0)
							{
								fixed_pixel[j] = 0;
							}
						}
					}
				}
			}
		}
	}
	name_fixed = "fixed_Mask_" + std::to_string(frame_index) + ".png";
	cv::namedWindow("fixed Mask", 1);
	cv::moveWindow("fixed Mask", 360, 400);
	cv::imshow("fixed Mask", fixed_Mask);
	const wchar_t* dir;
	dir = L"F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\fixed mask\\";
	if (_wmkdir(dir) != -1 && c_First_Time)
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\fixed mask\\" + name_fixed, fixed_Mask);
	}
	else
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\fixed mask\\" + name_fixed, fixed_Mask);
	}
	prev_cloud_Mask = frame.clone();
	prev_cloud_Mask = cloud_Mask;
}

void mountain(int height, int width)
{
	mountain_Mask = Mat(height, width, 0);
	mountain_Mask = 255;
	int sky_points = 0;
	int moun_points = 0;
	for (int i = 8; i < height - 8; i++)
	{
		uchar* mountain_pixel = mountain_Mask.ptr<uchar>(i);
		for (int j = 8; j < width - 8; j++)
		{
			if (cloud_Mask.at<uchar>(i, j) > 0 && fixed_Mask.at<uchar>(i, j) > 0)
			{
				for (int i1 = i - 8; i1 <= i + 8; i1++)
				{
					for (int j1 = j - 8; j1 <= j + 8; j1++)
					{
						if ((int)arr[i1][j1] <= ((int)arr[i][j] - C_sky))
						{
							if ((i1 == i) && (j1 == j)) continue;
							sky_points++;
						}
						if ((int)arr[i1][j1] >= ((int)arr[i][j] + C_mountain))
						{
							if ((i1 == i) && (j1 == j)) continue;
							moun_points++;
						}
					}
				}
				if (sky_points >= N_sky && moun_points >= N_mountain)
				{
					mountain_pixel[j] = 0;
				}
				else
				{
					mountain_pixel[j] = 255;
				}
			}
			else
			{
				mountain_pixel[j] = 255;
			}
			sky_points = 0;
			moun_points = 0;
		}
	}
	name_mountain = "mountain_Mask_" + std::to_string(frame_index) + ".png";
	cv::namedWindow("mountain", 1);
	cv::moveWindow("mountain", 700, 400);
	cv::imshow("mountain", mountain_Mask);
	const wchar_t* dir;
	dir = L"F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\mountain\\";
	if (_wmkdir(dir) != -1 && c_First_Time)
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\mountain\\" + name_mountain, mountain_Mask);
	}
	else
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\mountain\\" + name_mountain, mountain_Mask);
	}
}

void fill_target(int height, int width)
{
	bin_Mask = Mat(height, width, 0);
	bin_Mask = 0;
	std::ofstream target_list;
	int target_index = 0;
	string list_name = std::to_string(frame_index) + ".csv";
	const wchar_t* dir_xl;
	dir_xl = L"F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\xl_lists\\";
	if (_wmkdir(dir_xl) != -1 && c_First_Time)
	{
		target_list.open("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\xl_lists\\" + list_name, 0);
	}
	else
	{
		target_list.open("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\xl_lists\\" + list_name, 0);
	}
	for (int i = 8; i < height - 8; i++)
	{
		uchar* bin_pixel = bin_Mask.ptr<uchar>(i);
		for (int j = 8; j < width - 8; j++)
		{

			if (cloud_Mask.at<uchar>(i, j) > 0 && fixed_Mask.at<uchar>(i, j) > 0 && mountain_Mask.at<uchar>(i, j) > 0)
			{
				bin_pixel[j] = 255;
				target_Add(i, j, TARGET_RED, TARGET_RED);
				string detile = std::to_string(target_index) + "," + std::to_string(i) + "," + std::to_string(j) + "\n";
				target_list << detile;
				target_index++;
			}
		}
	}
	target_list.close();
	name_bin = "bin_Mask_" + std::to_string(frame_index) + ".png";
	cv::namedWindow("bin", 1);
	cv::moveWindow("bin", 1040, 400);
	cv::imshow("bin", bin_Mask);
	const wchar_t* dir;
	dir = L"F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\bin\\";
	if (_wmkdir(dir) != -1 && c_First_Time)
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\bin\\" + name_bin, bin_Mask);
	}
	else
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\bin\\" + name_bin, bin_Mask);
	}
}

void target_Add(int x, int y, int temp_type, int type)
{
	target add_tar;
	add_tar.p.x = x;
	add_tar.p.y = y;
	add_tar.Is_isolated = true;
	add_tar.temp_type = temp_type;
	add_tar.type = type;
	add_tar.prev_index = -1;
	add_tar.speed_x = 10e16;
	add_tar.speed_y = 10e16;
	add_tar.bad_pixel = false;
	add_tar.pervIS_target_in_Current = false;
	add_tar.target_print = 0;
	current_targets.push_back(add_tar);
}

void bad_pixel(int height, int width)
{
	bad_pixel_Mask = Mat(height, width, 0);
	bad_pixel_Mask = 255;
	if (!c_First_Time)
	{
		for (int i = 0; i < current_targets.size(); i++)
		{
			target current;
			current = current_targets[i];
			for (int j = 0; j < previous_targets.size(); j++)
			{
				target previous;
				previous = previous_targets[j];
				if (current.p.x == previous.p.x && current.p.y == previous.p.y)
				{
					bad_pixel_Mask.at<uchar>(current.p.x, current.p.y) = 0;
					current_targets[i].bad_pixel = true;
					previous_targets[j].bad_pixel = true;
				}
			}
		}
	}
	name_bad_pixel = "bad_pixel_" + std::to_string(frame_index) + ".png";
	cv::namedWindow("bad pixel", 1);
	cv::moveWindow("bad pixel", 1380, 400);
	cv::imshow("bad pixel", bad_pixel_Mask);
	const wchar_t* dir;
	dir = L"F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\bad pixel\\";
	if (_wmkdir(dir) != -1 && c_First_Time)
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\bad pixel\\" + name_bad_pixel, bad_pixel_Mask);
	}
	else
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\bad pixel\\" + name_bad_pixel, bad_pixel_Mask);
	}
}

void remove_bad_pixel()
{
	vector<target> current;
	for (int i = 0; i < current_targets.size(); i++)
	{
		if (current_targets[i].bad_pixel) continue;
		else 
		{
			current.push_back(current_targets[i]);
		}
	}
	vector<target> previous;
	for (int i = 0; i < previous_targets.size(); i++)
	{
		if (previous_targets[i].bad_pixel) continue;
		else
		{
			previous.push_back(previous_targets[i]);
		}
	}
	current_targets.clear();
	previous_targets.clear();
	for (int i = 0; i < current.size(); i++)
	{
		current_targets.push_back(current[i]);
	}
	for (int i = 0; i < previous.size(); i++)
	{
		previous_targets.push_back(previous[i]);
	}
	current.clear();
	previous.clear();
}

void scan_prev_in_current(int height, int width)
{
	prev_In_Current = Mat(height, width, 0);
	prev_In_Current = 0;
	int ndiff1 = 0;
	int ndiff2 = 0;
	int ndiff3 = 0;
	int N_th1 = first_Threshould * (1 - (first_Threshould_Per / 100));
	int N_diff1 = diff1 * (1 - (diff1_Per / 100));
	for (int k = 0; k < previous_targets.size(); k++)
	{
		target &prev_t = previous_targets[k];
		Point p = prev_t.p + Point(cv_x, cv_y);
		for (int i = p.x - 1; i <= p.x + 1; i++)
		{
			uchar* p_in_C = prev_In_Current.ptr<uchar>(i);
			for (int j = p.y - 1; j <= p.y + 1; j++)
			{
				if ((int)arr[i][j] >= N_th1 && (int)arr[i][j] <= second_Threshould)
				{
					for (int i1 = i - 1; i1 <= i + 1; i1++)
					{
						for (int j1 = j - 1; j1 <= j + 1; j1++)
						{
							if (((int)arr[i1][j1] < (int)arr[i][j] + N_diff1 / 2) && ((int)arr[i1][j1] > (int)arr[i][j] - N_diff1 / 2))
							{
								if ((i1 == i) && (j1 == j)) continue;
								ndiff1++;
							}
						}
					}
					for (int i2 = i - 2; i2 <= i + 2; i2++)
					{
						for (int j2 = j - 2; j2 <= j + 2; j2++)
						{
							if (((int)arr[i2][j2] < (int)arr[i][j] + N_diff1) && ((int)arr[i2][j2] > (int)arr[i][j] - N_diff1))
							{
								if (in_First(i, j, i2, j2)) continue;
								ndiff2++;
							}
						}
						if ((ndiff1 + ndiff2) > N_cloud)
						{
							p_in_C[j] = 0;
							break;
						}
					}
					ndiff3 = ndiff1 + ndiff2;
					if (ndiff3 <= N_cloud)
					{
						p_in_C[j] = 255;
					}
					else
					{
						p_in_C[j] = 0;
					}
				}
				else
				{
					p_in_C[j] = 0;
				}
				ndiff1 = 0;
				ndiff2 = 0;
				ndiff3 = 0;
			}
		}
	}
	name_prev_Incurrent = "prev_Incurrent_" + std::to_string(frame_index) + ".png";
	cv::namedWindow("prev Incurrent", 1);
	cv::moveWindow("prev Incurrent", 1720, 400);
	cv::imshow("prev Incurrent", prev_In_Current);
	const wchar_t* dir;
	dir = L"F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\prev In Current\\";
	if (_wmkdir(dir) != -1 && c_First_Time)
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\prev In Current\\" + name_prev_Incurrent, prev_In_Current);
	}
	else
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\prev In Current\\" + name_prev_Incurrent, prev_In_Current);
	}

}

void initilize_Targets_lists()
{
	// intitilize prev taregets
	if (!c_First_Time)
	{
		for (int i = 0; i < previous_targets.size(); i++)
		{

			Point q = previous_targets[i].p + Point(cv_x, cv_y);
			if (prev_In_Current.at<uchar>(q.x, q.y) > 0)
			{
				previous_targets[i].pervIS_target_in_Current = true;
			}
		}
	}
	if (current_targets.size() > 0)
	{
		for (int i = 0; i < current_targets.size(); i++)
		{
			Point p = current_targets[i].p;
			for (int i = p.x - C_r1; i <= p.x + C_r1; i++)
			{
				for (int j = p.y - C_r1; j <= p.y + C_r1; j++)
				{
					if ((i == j) && (i == 0) && (j == 0)) continue;
					if (cloud_Mask.at<uchar>(i, j) > 0)
					{
						current_targets[i].Is_isolated = false;
						break;
					}
				}
				if (!current_targets[i].Is_isolated) break;
			}
			calculate_target_Print(i);
		}
	}
}

void calculate_target_Print(int index)
{
	int sub = 0;
	int sum = 0;
	double print = 0;
	Point p = current_targets[index].p;
	for (int i = p.x - 2; i < p.x + 2; i++)
	{
		for (int j = p.y - 2; j < p.x + 2; j++)
		{
			sub = arr[i][j] - arr[p.x][p.y];
			sum += sub*sub;
		}
	}
	print = sqrt(sum);
	current_targets[index].target_print = print;
}

void temporary_decision()
{
	if (!c_First_Time)
	{
		for (int i = 0; i < current_targets.size(); i++)
		{
			target& current = current_targets[i];
			Point p = current.p;
			for (int j = 0; j < previous_targets.size(); j++)
			{
				target& previous = previous_targets[i];
				Point q = previous.p + Point(cv_x, cv_y);
				int dx = p.x - q.x;
				int dy = p.y - q.y;
				double distance = sqrt(dx*dx + dy*dy);
				if (current.Is_isolated)
				{
					if (distance <= (C_r1 + 0.5))
					{
						/*current.temp_type = TARGET_NONE;
						current.type = TARGET_NONE;*/
					}
					else
					{
						if ((dx > C_r1) && (dy > C_r1) && (dx <= C_dx) && (dy <= C_dy))
						{
							if (fixed_Mask.at<uchar>(q.x, q.y) == 0) break;
							if (mountain_Mask.at<uchar>(q.x, q.y) == 0) break;
							if (bad_pixel_Mask.at<uchar>(q.x, q.y) == 0) break;
							if (dx < current.speed_x)
							{
								current.temp_type = TARGET_GREEN;
								current.prev_index = j;
								current.speed_x = dx;
								current.speed_y = dy;
							}
						}
						else if ((dx <= C_r1) && (dy > C_r1) && (dy <= C_dy))
						{
							if (fixed_Mask.at<uchar>(q.x, q.y) == 0) break;
							if (mountain_Mask.at<uchar>(q.x, q.y) == 0) break;
							if (bad_pixel_Mask.at<uchar>(q.x, q.y) == 0) break;
							if (dx < current.speed_x)
							{
								current.temp_type = TARGET_GREEN;
								current.prev_index = j;
								current.speed_x = dx;
								current.speed_y = dy;
							}
						}
					}
				}
				else
				{
					if (distance <= (C_r1 + 0.5))
					{
						/*current.temp_type = TARGET_NONE;
						current.type = TARGET_NONE;*/
					}
					else
					{
						if ((dx > C_r1) && (dy > C_r1) && (dx <= C_dx) && (dy <= C_dy))
						{
							current.temp_type = TARGET_GREEN;
							current.prev_index = j;
							current.speed_x = dx;
							current.speed_y = dy;
						}
						else if ((dx <= C_r1) && (dy > C_r1) && (dy <= C_dy))
						{
							current.temp_type = TARGET_GREEN;
							current.prev_index = j;
							current.speed_x = dx;
							current.speed_y = dy;
						}
					}
				}
			}
		}
	}
}

void final_decision()
{
	if (!c_First_Time)
	{
		for (int i = 0; i < current_targets.size(); i++)
		{
			
			target& current = current_targets[i];
			target prev = previous_targets[current_targets[i].prev_index];
			if (current.temp_type == TARGET_GREEN)
			{
				if (prev.pervIS_target_in_Current)
				{
					current.type = TARGET_RED;
				}
				else if (((prev.target_print - current.target_print) / prev.target_print) > 0.1)
				{
					current.type = TARGET_RED;
				}
				/*else if (current.speed_x <= 1)
				{
					current.type = TARGET_NONE;
				}*/
				else
				{
					current.type = TARGET_GREEN;
				}
			}
		}
	}
}

void draw_results()
{	
	Mat result = frame;
	for (int i = 0; i < current_targets.size(); i++)
	{
		Rect rec = Rect(current_targets[i].p.y - 10, current_targets[i].p.x - 10, 20, 20);
		if(current_targets[i].type == TARGET_GREEN)
		{
			cv::rectangle(result, rec, cv::Scalar(0, 255, 0), 1, 8, 0);
		}
		else if (current_targets[i].type == TARGET_RED)
		{
			cv::rectangle(result, rec, cv::Scalar(0, 0, 255), 1, 8, 0);
		}
		else if (current_targets[i].type == TARGET_NONE)
		{
			cv::rectangle(result, rec, cv::Scalar(0, 255, 255), 1, 8, 0);
		}
	}	
	cv::namedWindow("result", 1);
	name_result = "result_" + std::to_string(frame_index) + ".png";
	//cv::moveWindow("result", 20, 400);
	cv::imshow("result", result);
	const wchar_t* dir;
	dir = L"F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\result\\";
	if (_wmkdir(dir) != -1 && c_First_Time)
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\result\\" + name_result, result);
	}
	else
	{
		cv::imwrite("F:\\target\\Drone_Detection\\Drone_Detection\\Drone_Detection\\A_result\\result\\" + name_result, result);
	}
}

void current_2_previous()
{
	if (!c_First_Time)
	{
		previous_targets.clear();
	}
	for (int i = 0; i < current_targets.size(); i++)
	{
		if (current_targets[i].type == TARGET_NONE) continue;
		previous_targets.push_back(current_targets[i]);
	}
	current_targets.clear();
	delete arr;
}


int APIENTRY wWinMain(_In_ HINSTANCE hInstance, _In_opt_ HINSTANCE hPrevInstance, _In_ LPWSTR    lpCmdLine, _In_ int nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);
	LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
	LoadStringW(hInstance, IDC_FACE_RECOGNITION, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);
	if (!InitInstance(hInstance, nCmdShow))
	{
		return FALSE;
	}
	HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_FACE_RECOGNITION));
	MSG msg;
	while (GetMessage(&msg, nullptr, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}
	return (int)msg.wParam;
}

ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEXW wcex;
	wcex.cbSize = sizeof(WNDCLASSEX);
	wcex.style = CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc = WndProc;
	wcex.cbClsExtra = 0;
	wcex.cbWndExtra = 0;
	wcex.hInstance = hInstance;
	wcex.hIcon = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_FACE_RECOGNITION));
	wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
	wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	wcex.lpszMenuName = NULL;
	wcex.lpszClassName = szWindowClass;
	wcex.hIconSm = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
	return RegisterClassExW(&wcex);
}

BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
	hInst = hInstance;
	HWND hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, 0, 300, 200, nullptr, nullptr, hInstance, nullptr);
	if (!hWnd) { return FALSE; }
	ShowWindow(hWnd, nCmdShow);
	UpdateWindow(hWnd);
	return TRUE;
}

LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_CREATE:
	{
		capture_Img_btn = CreateWindow(
			L"BUTTON",
			L"OPEN VIDEO ",
			WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,
			25, 30, 150, 40,
			hWnd,
			(HMENU)IDC_CAPTURE_BTN,
			hInst,
			0);
		feature_Save_btn = CreateWindow(
			L"BUTTON",
			L"CLOSE",
			WS_TABSTOP | WS_VISIBLE | WS_CHILD | BS_DEFPUSHBUTTON,
			25, 80, 150, 40,
			hWnd,
			(HMENU)IDC_FEATURE_SAVE_BTN,
			hInst,
			0);
	}
	case WM_COMMAND:
	{
		int wmId = LOWORD(wParam);
		switch (wmId)
		{
		case IDC_CAPTURE_BTN:
		{
			OPENFILENAME ofn;
			TCHAR szFilter[] = TEXT("video (*.avi)\0*.avi\0")\
				TEXT("video (*.mp4)\0*.mp4\0")\
				TEXT("All Files (*.*)\0*.*\0\0");
			TCHAR NameOfFile[MAX_PATH], TitleOfFile[MAX_PATH];
			NameOfFile[0] = '\0';
			TitleOfFile[0] = '\0';
			ofn.lStructSize = sizeof(OPENFILENAME);
			ofn.hwndOwner = feature_Save_btn;
			ofn.hInstance = NULL;
			ofn.lpstrFilter = szFilter;
			ofn.lpstrCustomFilter = NULL;
			ofn.nMaxCustFilter = 0;
			ofn.nFilterIndex = 0;
			ofn.lpstrFile = NameOfFile;
			ofn.nMaxFile = MAX_PATH;
			ofn.lpstrFileTitle = TitleOfFile;
			ofn.nMaxFileTitle = MAX_PATH;
			ofn.lpstrInitialDir = NULL;
			ofn.lpstrTitle = NULL;
			ofn.Flags = OFN_HIDEREADONLY | OFN_CREATEPROMPT;
			ofn.nFileOffset = 0;
			ofn.nFileExtension = 0;
			ofn.lpstrDefExt = TEXT("rtf");
			ofn.lCustData = 0L;
			ofn.lpfnHook = NULL;
			ofn.lpTemplateName = NULL;
			int getReturnValue = GetOpenFileName(&ofn);
			wstring test(&NameOfFile[0]);
			string video_path(test.begin(), test.end());
			capture_Video(video_path);
			break;
		}
		case IDC_FEATURE_SAVE_BTN:
		{
			break;
		}
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}
		break;
	}
	case WM_PAINT:
	{
		{
			PAINTSTRUCT ps;
			HDC hdc = BeginPaint(hWnd, &ps);
			EndPaint(hWnd, &ps);
		}
		break;
	}
	case WM_CLOSE:
	{
		if (MessageBox(hWnd, L"Really quit?", L"Drone_Detection", MB_OKCANCEL) == IDOK)
		{
			PostQuitMessage(0);
		}
	}
	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}
