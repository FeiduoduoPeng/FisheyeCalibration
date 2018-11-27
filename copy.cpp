 vector<Point2f> img_ptsTemp;
 vector<vector<Point2f>> img_ptsVector;
 
 
 vector<Point3f> obj_ptsTemp;
 vector<vector<Point3f>> obj_ptsVector;
 
  // 鱼眼镜头参数
 Mat intrinsic_mat;            //Matx33d intrinsic_mat亦可;           Mat intrinsic_mat(3, 3, CV_64FC1, Scalar(0))亦可，注意数据类型;      
 Mat distortion_coeffs;     //Vec4d distortion_coeffs亦可

 int flag = 0;
 flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
 flag |= cv::fisheye::CALIB_CHECK_COND;
 flag |= cv::fisheye::CALIB_FIX_SKEW;  /*非常重要*/

cv::fisheye::calibrate(
    obj_ptsVector,
    img_ptsVector,
    board_img_size,
    intrinsic_mat, 
    distortion_coeffs,
    cv::noArray(),      
    cv::noArray(),     
    flag,
    cv::TermCriteria(3, 20, 1e-6)
);



//畸变校正

void FishEyeImgUndistort()
{
    Mat DistortImg = cv::imread("fore_1.jpg");

    Mat UndistortImg;   
    Mat new_intrinsic_mat;    //Mat new_intrinsic_mat(3, 3, CV_64FC1, Scalar(0))亦可，注意数据类型;


    //fx,fy变大（小），视场变小（大），裁剪较多（少），但细节清晰（模糊）；很关键，new_intrinsic_mat决定输出的畸变校正图像的范围
    intrinsic_mat.copyTo(new_intrinsic_mat);

    //调整输出校正图的视场
    new_intrinsic_mat.at<double>(0, 0) *= 0.5;      //注意数据类型，非常重要
    new_intrinsic_mat.at<double>(1, 1) *= 0.4; 


    //调整输出校正图的中心
    new_intrinsic_mat.at<double>(0, 2) += 0.0;   
    new_intrinsic_mat.at<double>(1, 2) += 0.0;


    cv::fisheye::undistortImage(
    DistortImg,
    UndistortImg,
    intrinsic_mat,
    distortion_coeffs,
    new_intrinsic_mat    //
    );      //最后一个camera_matrix必须写上  Ref:http://answers.opencv.org/question/64614/fisheyeundistortimage-doesnt-work-what-wrong-with-my-code/


    cv::imshow("DistortImg", DistortImg);
    cv::imshow("UndistortImg", UndistortImg);
    cv::imwrite("feModelUndistortFore.jpg", UndistortImg);
    cv::waitKey(0);
}
