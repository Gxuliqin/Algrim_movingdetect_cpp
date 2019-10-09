#ifndef __OPTICAL__H__
#define __OPTICAL__H__

#pragma once
#include<opencv2/video/video.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<iostream>
#include<cstdio>
#include<stdlib.h>
#include<map>

using namespace std;
using namespace cv;

//--------声明全局函数------------
void tracking(Mat &frame,Mat &output, Mat &affineoutput);
bool addNewPoints();
bool acceptTrackedPoint(int i);
bool findmaxpoint(map<int, vector<Point2f>> &ppoe, vector<Point2f> initial, float &a, float &b);
bool computemeanxy(vector<Point2f> &xy_vector, float &a, float &b);
void videostabilize(Mat &input, Mat &output);
//bool findmaxpoint(vector<vector<Point2f>> &ppoe, float &a, float &b)
//--------声明全局变量------------
static string window_name = "optical flow tracking 光流检测";
static Mat gray;		//当前图片
static Mat gray_prev;	//预测图片

static vector<Point2f> points[2];		//point0为特征点的原来位置，point1为特征点的新位置
static vector<Point2f> initial;		//初始化跟踪点的位置
static vector<Point2f> features;		//检测的特征
static int maxCount = 500;				//检测的最大特征数
static double qLevel = 0.01;			//特征检测的等级
static double minDest = 1.0;			//两特征点之间最小的距离
static vector<uchar> status;			//跟踪特征的状态，特征的流发现为1，否则为0
static vector<float> err;

//--------help函数 打印程序的说明-----------
void help();
#endif
