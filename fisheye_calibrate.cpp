#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <stdlib.h>
#include <stdio.h>

#include <fstream>
#include <dirent.h>

#define WIDTH 10
#define HEIGHT 11
#define squareSize 40

using namespace std;
using namespace cv;

void FishEyeImgUndistort();

int main(int argc, char const *argv[])
{
    DIR * dir;
    struct dirent * ptr;
    string rootdirPath = "../";
    string x,dirPath;
    vector<string> filenames;
    dir = opendir((char *)rootdirPath.c_str()); //打开一个目录
    while((ptr = readdir(dir)) != NULL) //循环读取目录数据
    {
        x=ptr->d_name;
        dirPath = rootdirPath + x;
        if( string::npos == dirPath.find(".jpg")){
            continue;
        }
        //std::cout<<dirPath<<std::endl;
        filenames.push_back(dirPath);
    }
    closedir(dir);//关闭目录指针
    cout<<"found images number: "<<filenames.size()<<endl;

    vector<cv::Point3f> obj;
    for(int i=0; i<HEIGHT; ++i){
        for(int j=0; j<WIDTH; ++j){
            obj.push_back( cv::Point3f(float(j*squareSize),
                                        float(i*squareSize), 0 ));
        }
    }

    vector<Mat> pics;
    cv::Mat gray_pic, color_pic;
    vector<cv::Point2f> corners;
    vector<vector<cv::Point2f> > imgPoints;
    Size sz = imread(filenames[0]).size();
    cout<<"size: "<<sz<<endl;
    int numl=0;
    for(int i=0; i<filenames.size(); ++i){
        color_pic = cv::imread(filenames[i]);
        cv::cvtColor(color_pic, gray_pic, cv::COLOR_RGB2GRAY);
        int found = findChessboardCorners(color_pic, Size(WIDTH,HEIGHT), corners);
        if(found){
            cv::cornerSubPix(gray_pic, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1) );
            imgPoints.push_back(corners);

            //drawChessboardCorners(color_pic, Size(WIDTH,HEIGHT), cv::Mat(corners), true);
            //imshow("chessboard", color_pic);
            //waitKey(1);
            //numl++;

            corners.clear();
        }
    }
    vector<vector<cv::Point3f> > objPoints(imgPoints.size(), obj);
    //cout<<"numl size: "<<numl<<endl;
    //cout<<"imgPoints size: "<<imgPoints.size()<<endl;
    //cout<<"objPoints size: "<<objPoints.size()<<endl;

    cv::Mat K;
    cv::Mat D;
    vector<cv::Mat> rvecs, tvecs;
    int flag = 0;
    flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
    //flag |= cv::fisheye::CALIB_CHECK_COND;
    flag |= cv::fisheye::CALIB_FIX_SKEW; 

    double rms = cv::fisheye::calibrate(
        objPoints, imgPoints, sz,
        K, D, cv::noArray(), cv::noArray(), 
        flag,
        cv::TermCriteria(3,20,1e-6)
    );
    
    cout<<"K: "<<endl<<K<<endl;
    cout<<"D: "<<endl<<D<<endl;
    cout<<"rms: "<<rms<<endl;

    Mat UndistortImg;
    Mat DistortImg = cv::imread("../img28.jpg");

    Mat new_intrinsic_mat;
    K.copyTo(new_intrinsic_mat);
    //new_intrinsic_mat.at<double>(0,0) *= 0.5;
    //new_intrinsic_mat.at<double>(1,1) *= 0.4;
    new_intrinsic_mat.at<double>(0,2) += 0.0;
    new_intrinsic_mat.at<double>(1,2) += 0.0;

    cv::fisheye::undistortImage(
        DistortImg,
        UndistortImg,
        K,
        D,
        new_intrinsic_mat
    );
    imshow("undistort", UndistortImg);
    waitKey(0);
    return 0;
}
