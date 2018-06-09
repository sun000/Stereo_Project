//
//  main.cpp
//  Stereo_project
//
//  Created by 孙辉 on 2018/6/5.
//  Copyright © 2018年 孙辉. All rights reserved.
//

#include <string>
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "opencv2/opencv.hpp"

#define mDEBUG false

using namespace cv;
using namespace std;

vector<Mat> readData(String listPath, String picturePath) {
    ifstream fin(listPath);
    if(!fin) {
        printf("list file is'n exist!");
        exit(-1);
    }
    string filename;
    vector<Mat> images;
    Mat image;
    while(fin >> filename) {
        image = imread(picturePath + "/" + filename);
        if(image.empty()) cout << picturePath + "/" + filename + "is't exist!" <<endl;
        else images.push_back(image);
    }
    return images;
}

void showAllPicture(vector<Mat> &images) {
    namedWindow("Origin Picture");
    for(int i = 0; i < images.size(); i++) {
        imshow("Origin Picture", images[i]);
        waitKey(500); //hold 0.5s
    }
    destroyWindow("Origin Picture");
}

vector<vector<Point2f> > getCorners(const vector<Mat> &images, Size patternSize) {
    Size imageSize;
    imageSize.height = images[0].rows;
    imageSize.width  = images[0].cols;
    
    vector<vector<Point2f> > cornersList;
    if(mDEBUG) namedWindow("Camera Calibration");//mDEBUG
    for(int i = 0; i < images.size(); i++) {
        vector<Point2f> outPutCorners;
        if(findChessboardCorners(images[i], patternSize, outPutCorners)) {//粗略检测是否有角点
            //亚像素级检测
            Mat grayImage;//得到灰度图
            cvtColor(images[i], grayImage, CV_RGB2GRAY);
            //参数按顺序分别是：输入图像 输出角点 搜索窗口 死区窗口 终止条件（集合迭代次数和角点精确度）
            cornerSubPix(grayImage, outPutCorners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            cornersList.push_back(outPutCorners);
            if(mDEBUG) {
                drawChessboardCorners(images[i], patternSize, outPutCorners, false);
                imshow("Camera Calibration", images[i]);
                waitKey(500);
            }
        }
    }
    if(mDEBUG) destroyWindow("Camera Calibration"); //mDEBUG
    return cornersList;
}

void mCalibrate(Mat image, const vector<vector<Point2f> > &cornersList, Size imageSize, Size patternSize, Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecsMat, vector<Mat>& tvecsMat) {
    
    //将所有的角点映射到世界坐标中, 假设z=0,并找到所有角点
    vector<Point3f> pointInWorld;
    double square_len = (cornersList[0][1].x - cornersList[0][0].x);
    for(int i = 0; i < patternSize.height; i++)
        for(int j = 0; j < patternSize.width; j++)
            pointInWorld.push_back(Point3f((i + 1) * square_len, (j + 1) * square_len, 0));
    vector<vector<Point3f> > cornersInTheWorld(cornersList.size(), pointInWorld);
    //参数以此表示：世界坐标系汇总的角点坐标 角点对应的二位坐标 内参矩阵 畸变系数 旋转向量 平移向量
    calibrateCamera(cornersInTheWorld, cornersList, imageSize, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
}

int main(void) {
    //读取数据
    vector<Mat> images = readData("fileList.txt", "left");
    if(mDEBUG) showAllPicture(images); //显示图片
    Size imageSize;
    imageSize.height = images[0].rows;
    imageSize.width  = images[0].cols;
    //检索角点
    Size patternSize = Size(9, 6); //角点规模
    vector<vector<Point2f> > cornersList = getCorners(images, patternSize);
    
    //待标定参数
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));  //intrinsics
    Mat distCoeffs = Mat(1, 4, CV_32FC1,Scalar::all(0));       //extrinsics k1,k2,p1,p2
    vector<Mat> rvecsMat, tvecsMat;     //R 旋转向量 t 平移向量
    //标定
    mCalibrate(images[0], cornersList, imageSize, patternSize, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
//    修正失真
    namedWindow("tmp");
    Mat ImgUndistort = images[0].clone(); //undistort(Img,ImgUndistort,CM,D);
    string resultPath = "results/";
    for(int i = 0; i < images.size(); i++) {
        undistort(images[i], ImgUndistort, cameraMatrix, distCoeffs);
        imshow("tmp", ImgUndistort);
        waitKey(500);
        imwrite(resultPath + "/left" + to_string(i + 1) + ".jpg" , ImgUndistort);
    }
    destroyWindow("tmp");
    return 0;
}
