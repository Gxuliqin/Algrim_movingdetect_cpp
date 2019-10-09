#include<opencv2/video/video.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<iostream>
#include<cstdio>
#include<stdlib.h>
#include<map>
#include"optical.h"
using namespace std;
using namespace cv;


void help(){
cout<<"OpenCV version :"<<CV_VERSION<<endl;
}

//-------main函数 程序入口-------------------
// int main()
// {
// 	Mat frame;
// 	Mat result;
//     const string url = "rtsp://admin:password@192.168.1.104:554/11";
// 	//VideoCapture capture(0);
//     VideoCapture capture("2.mp4");
// 	help();
// 
// 	if (capture.isOpened())
// 	{
// 		while (true)
// 		{
// 			capture >> frame;
// 			if (!frame.empty())             //不为空
// 			{
// 				tracking(frame, result);     //跟踪
// 			}
// 			else
// 			{
// 				printf("No Capture frame , Break");
// 				break;
// 			}
// 			int c = waitKey(50);
// 			if (27 == (char)c)
// 			{
// 				break;
// 			}
// 		}
// 	}
// 
// 	return 0;
// }
//--------tracking函数  跟踪运动目标------------------
//	frame：输入的视频帧         output:有跟踪结果的视频帧
void videostabilize(Mat &frame, Mat &output){
    Mat gray;
    cvtColor(frame, gray, CV_BGR2GRAY);
    if (addNewPoints()){
        goodFeaturesToTrack(gray, features, maxCount, qLevel, minDest);
		points[0].insert(points[0].end(), features.begin(), features.end());
		initial.insert(initial.end(), features.begin(), features.end());
    }    
    if (gray_prev.empty())
	{
		gray.copyTo(gray_prev);
	}
	//l-k流光法运动估计
	calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err);
    vector<Point2f> xy_vector;
    map<int, vector<Point2f>> ppoe;
    int k = 0;
	for (size_t i = 0; i < points[1].size(); i++)
	{
		if (acceptTrackedPoint(i))
		{
			initial[k] = initial[i];
			points[1][k++] = points[1][i];
		}
	}
    points[1].resize(k);
	initial.resize(k);
	for (size_t i = 0; i < points[1].size(); i++)
	{
        line(output, initial[i], points[1][i], Scalar(0, 0, 255));
		circle(output, points[1][i], 4, Scalar(0, 255, 0), -1);
        circle(output, points[0][i], 4, Scalar(255, 0, 0), -1);
        Point2f abs_xy;
        abs_xy.x = points[1][i].x- initial[i].x;
        abs_xy.y = points[1][i].y- initial[i].y;
        float dst = sqrt(abs_xy.x*abs_xy.x + abs_xy.y*abs_xy.y);
        cout<<"dst is: "<<dst<<" ,x is: "<<abs_xy.x<<" ,y is: "<<abs_xy.y<<endl;
        xy_vector.push_back(abs_xy);
        //ppoe[int(dst)] = temp;
	}
	
	float a=0, b=0;
	//findmaxpoint(ppoe, initial, a, b);
    computemeanxy(xy_vector, a, b);
	Mat affinemat;
    affinemat = (Mat_<double>(2,3)<<1, 0, -a, 0, 1, -b);
    warpAffine(frame, output, affinemat, frame.size());
}

void tracking(Mat &frame, Mat &output, Mat &affineoutput)
{
	cvtColor(frame, gray, CV_BGR2GRAY);
	frame.copyTo(output);

	//添加特征点
	if (addNewPoints())
	{
		goodFeaturesToTrack(gray, features, maxCount, qLevel, minDest);
		points[0].insert(points[0].end(), features.begin(), features.end());
		initial.insert(initial.end(), features.begin(), features.end());
	}
	if (gray_prev.empty())
	{
		gray.copyTo(gray_prev);
	}
	//l-k流光法运动估计
	calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err);
	//去掉一些不好的特征点
	//int k = 0;
	//for (size_t i = 0; i < points[1].size(); i++)
	//{
	//	if (acceptTrackedPoint(i))
	//	{
	//		initial[k] = initial[i];
	//		points[1][k++] = points[1][i];
	//	}
	//}
    //points[1].resize(k);
	//initial.resize(k);
	cout<<"origin point: "<<points[0].size()<<"then: "<<points[1].size()<<endl;
	
	//显示特征点和运动轨迹
    vector<Point2f> xy_vector;
    map<int, vector<Point2f>> ppoe;
	for (size_t i = 0; i < points[1].size(); i++)
	{
        
        Point2f abs_xy;
		line(output, initial[i], points[1][i], Scalar(0, 0, 255));
		circle(output, points[1][i], 3, Scalar(0, 255, 0), -1);
        circle(output, points[0][i], 3, Scalar(255, 0, 0), -1);
        abs_xy.x = points[1][i].x- initial[i].x;
        abs_xy.y = points[1][i].y- initial[i].y;
        float dst = sqrt(abs_xy.x*abs_xy.x + abs_xy.y*abs_xy.y);
        cout<<"dst is: "<<dst<<" ,x is: "<<abs_xy.x<<" ,y is: "<<abs_xy.y<<endl;
        xy_vector.push_back(abs_xy);
        //ppoe[int(dst)] = temp;
	}
	float a=0, b=0;
	//findmaxpoint(ppoe, initial, a, b);
    computemeanxy(xy_vector, a, b);
	Mat affinemat;
    affinemat = (Mat_<double>(2,3)<<1, 0, -a, 0, 1, -b);
    warpAffine(output, affineoutput, affinemat, affineoutput.size());
    cout<<"juz: "<<affinemat<<endl<<endl;
	//把当前跟踪结果作为下一次的参考
	//swap(points[1],points[0]);
	//swap(gray_prev,gray);
	imshow(window_name, output);
    imshow("affine", affineoutput);
}

bool computemeanxy(vector<Point2f> &xy_vector, float &a, float &b){
    float sum_x=0, sum_y=0;
    for (auto perpoint:xy_vector){
        sum_x += perpoint.x;
        sum_y += perpoint.y;
    }
    a = sum_x/xy_vector.size();
    b = sum_y/xy_vector.size();
    return true;
}

bool findmaxpoint(map<int, vector<Point2f>> &ppoe, vector<Point2f> initial, float &a, float &b){
    int max;
    int i = 0;
    vector<Point2f> point;
    for (auto x:ppoe){
        if (i ==0) {max = x.second.size(); point = x.second;}
        if(x.second.size()>max){
            max = x.second.size();
            point = x.second;
            cout<<"ok"<<endl;
        }
        i++;
    }
    cout<<"max matched point is :"<<point.size()<<endl;
    float sumx, sumy;
    sumx = 0;
    sumy = 0;
    for (auto perpoint:point){
        sumx += (perpoint.x - initial[i].x);
        sumy += (perpoint.y - initial[i].y);
    }
    a = sumx/point.size();
    b = sumy/point.size();
    cout<<"point size: "<<point.size()<<endl;
    return true;
}

//--------addNewPoints函数 ：检测新店是否应该被添加----------------
//	return是否被添加的标志

bool addNewPoints()
{
	return points[0].size() <= 10;       //points.size()求行数     points.size()求列数

}

//-------acceptTrackedPoint函数：决定哪些跟踪点被接收-------------
bool acceptTrackedPoint(int i)
{
	return status[i] && ((abs(points[0][i].x - points[1][i].x) + abs(points[0][i].y - points[1][i].y)) > 1);
}

