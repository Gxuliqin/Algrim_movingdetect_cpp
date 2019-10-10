#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include<time.h>
#include<stdlib.h>
#include<sstream>
#include<vector>
#include "optical.h"
#include "estimation.h"

using namespace cv;
using namespace std;
void threshould(Mat &frame1, Mat &frame0, Mat &disfame){
//     Mat disfame(frame1.rows, frame1.cols, CV_8UC1);
    for (size_t i=0; i<frame1.rows; i++){
        for(size_t j=0; j<frame1.cols; j++){
            if (frame1.at<uchar>(i,j) >= frame0.at<uchar>(i,j)){
                disfame.at<uchar>(i,j) = frame1.at<uchar>(i,j) - frame0.at<uchar>(i,j);
            }else{
                disfame.at<uchar>(i,j) = frame0.at<uchar>(i,j) - frame1.at<uchar>(i,j);
            }
        }
    }
}

void initial_background(Mat &dstframe, Mat &samples){
    samples.create(dstframe.rows, dstframe.cols, CV_8UC(20));
    int a = -1;
    int b = 1;
    int rani, ranj;
    int ri, rj;
    for (size_t i=0;i<samples.rows;i++){
        unsigned char* row_ptr = samples.ptr<unsigned char> ( i ); 
        for (size_t j=0;j<samples.cols;j++){
            unsigned char* data_ptr = &row_ptr[ j*samples.channels() ];
            for (size_t n=0;n<samples.channels();n++){
                rani = (rand() % (b-a+1))+ a;
                ranj = (rand() % (b-a+1))+ a;
                ri = i + rani;
                rj = j + ranj;
                data_ptr[n] = dstframe.at<uchar>(ri,rj);
            }
        }
    }
}

void vibe_detection(Mat &dstframe, Mat &samples, Mat &segmap, int _min, int R, int s){
    float dist;
    int count=0;
    
     for (size_t i=0;i<samples.rows;i++){
        unsigned char* row_ptr = samples.ptr<unsigned char> ( i ); 
        for (size_t j=0;j<samples.cols;j++){
            unsigned char* data_ptr = &row_ptr[ j*samples.channels() ];
            for (size_t n=0;n<samples.channels();n++){
               dist = abs(data_ptr[n] - dstframe.at<uchar>(i,j));
               if (dist<R){
                   count++;
                }
             }
             
            if (count>=_min){
                int a=0;
                int b=20;
                int randn, randx, randi, randj, randn_;
                randn = (rand() % (b-a+1))+ a;
                randx = (rand() % (s-a+1))+ a;
                if (randx == 1){
                data_ptr[randn] = dstframe.at<uchar>(i,j);
                }   //更新背景 
                if (randx ==2){
                    randi = (rand() % (1+1+1))-1;
                    randj = (rand() % (1+1+1))-1;
                    randn_ = (rand() % (20-0+1))+0;
                    if ( ((i+randi)<dstframe.rows )&&((j+randj)<dstframe.cols) ){
                    *(samples.ptr<uchar>(i+randi)+(j+randj)*samples.channels()+randn_) = dstframe.at<uchar>(i,j);
                    }
                }
            }else{
                segmap.at<uchar>(i,j)=255;
            }
        count = 0;
        }
    }
}

string itos(int i) // 将int 转换成string{
 {
    stringstream s;
    s << i;
    return s.str();
 }
 
int * find4index(vector<Point> vec){
    static int m[4];
    int max_x, max_y, min_x, min_y, i;
    i = 0;
    for(auto x:vec){
        if ( i == 0){
            max_x = x.x;
            min_x = x.x;
            max_y = x.y;
            min_y = x.y;
        }else{
            if (x.x>max_x) max_x = x.x;
            if (x.x<min_x) min_x = x.x;
            if (x.y>max_y) max_y = x.y;
            if (x.x>min_y) min_y = x.y;
        }
        i++;
    }
   
    m[2] = max_x - min_x;
    m[3] = max_y - min_y; 
    m[0] = min_x;
    m[1] = min_y;
  return m;   
}
 
 
bool findmaxcontours(vector<vector<Point>> &contours, vector<Point> &max_contours){
    int max=0;
    for (auto cont : contours ){
        if (cont.size() ==1) continue;
        if (cont.size()>max){
            max = cont.size();
            max_contours = cont;
        }
    }
    return true;
}
 
int main ( int argc, char** argv )
{    int i=0;
    Mat frame;
    int _min, N, R;
    N = 20;
    R = atoi(argv[2]);
    _min = 2;
    int s = atoi(argv[3]);
    Mat dstframe, frame0, frame1, disframe, _;
    Mat samples;
    VideoCapture cap(atoi(argv[1]));
    cout<<argv[1]<<endl;
   if (!cap.isOpened()){
       cout<<"视频打开失败"<<endl;
    }
    help();
    while(1){
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        cap>>frame;
        if (frame.empty()){
            break;
        }
        i++;
        
        //waitKey(20); 
        cvtColor(frame, dstframe, CV_BGR2GRAY);
        //tracking(frame, _, dstframe);
        //videostabilize(frame, dstframe);
      
        
        imshow("stabilize", dstframe);
        if (i==1){
            frame0 = frame;
            initial_background(dstframe, samples);
            continue;
        }
        estabilize(frame0, frame, dstframe);
        Mat segmap = Mat::zeros(dstframe.rows, dstframe.cols, CV_8UC1);
        vibe_detection(dstframe, samples, segmap, _min, R, s);
        
        Mat outmap, outmapc3;
        medianBlur(segmap, outmap, 3);
        //rectangle(frame, Rect(frame.cols/2-50, frame.rows/2-50, 100, 100), Scalar(0, 0, 255), 2, 8 );
        imshow("capturevide",frame);
        imshow("beforblur",segmap);
        //imshow("222", dstframe);
        
        Mat bimage;
        threshold(outmap, bimage, 100, 255,THRESH_BINARY);
        vector<vector<Point>> contours;
        findContours(bimage, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
        vector<Point> max_contours;
        findmaxcontours(contours, max_contours);
        cout<<"max_contours is:"<<max_contours.size()<<endl;
        int *p;
        p = find4index(max_contours);
        cvtColor(outmap, outmapc3, CV_GRAY2RGB);
        rectangle(outmapc3, Rect(*p, *(p+1), *(p+2), *(p+3)), Scalar(0, 0, 255), 2, 8);
        cout<<"rectangle :"<<*p<<" "<<*(p+1)<<" "<<*(p+2)<<" "<<*(p+3)<<endl;
        
        imshow("vibe",outmapc3);
        //Mat element,Dilate,Erode,morph_op;
        //element=getStructuringElement(MORPH_RECT,Size(2,2));
        //dilate(segmap,Dilate,element);
        //erode(segmap,Erode,element);
        //morphologyEx(segmap, morph_op, MORPH_OPEN, element);
        //imshow("dilate",Dilate);
        //imshow("erode",Erode);
        //imshow("open", morph_op);

        
        if ((argv[4]!=NULL)&&(argv[5]!=NULL)){
            imwrite(argv[4], frame);
            imwrite(argv[5], outmap);
        }
        char k;
        k = waitKey(20);
        if (k == 'q'){
            break;
        }
        if (k=='s'){
            int t = clock();

            string origin_path, seg_path, blur_path;
            origin_path.append("img/").append(itos(t)).append("origin.jpg");
            seg_path.append("img/").append(itos(t)).append("seg.jpg");
            blur_path.append("img/").append(itos(t)).append("blur.jpg");
                    
            imwrite(origin_path, dstframe);
            imwrite(seg_path, segmap);
            imwrite(blur_path, outmapc3);
            cout<<"saved success"<<endl;
        }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"遍历图像用时："<<time_used.count()<<" 秒。"<<endl;
    }
    cap.release();
    return 0;
}
