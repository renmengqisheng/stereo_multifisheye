/*================================================================
*   Copyright (C) 2019 * Ltd. All rights reserved.
*
*   Editor      : VIM
*   File name   : multi_fisheye_param.h
*   Author      : Mofan
*   Created date: 2019-05-29 16:32:55
*   Description :
*
*===============================================================*/

#pragma once

#include "Frame.h"
#include "ORBmatcher.h"
#include <iostream>
#include <string>

enum stereo_pair{
    left_to_front = 0,
    front_to_right,
    right_to_back,
    back_to_left
};

class multi_fisheye_param
{
public:
    void write(std::string fileName);
    void read(std::string fileName);

    multi_fisheye_param();
    multi_fisheye_param(std::string fileName);

    float ZNCC(cv::Mat img1, cv::Mat img2, cv::Point2i point1, cv::Point2i point2, int size);
    void ComputeStereoMatches(ORB_SLAM2::Frame& frame1, ORB_SLAM2::Frame& frame2, enum stereo_pair flag);
    void ComputeStereoMatches(ORB_SLAM2::Frame& left_frame, ORB_SLAM2::Frame& front_frame, ORB_SLAM2::Frame& right_frame, ORB_SLAM2::Frame& back_frame);

public:
    cv::Mat left_mapx;
    cv::Mat left_mapy;

    cv::Mat front_mapx;
    cv::Mat front_mapy;

    cv::Mat right_mapx;
    cv::Mat right_mapy;

    cv::Mat back_mapx;
    cv::Mat back_mapy;

    double fx;
    double fy;
    double cx;
    double cy;
    double ThDepth;

    double left_baseline;
    double front_baseline;
    double right_baseline;
    double back_baseline;

    double left_ThDepth;
    double front_ThDepth;
    double right_ThDepth;
    double back_ThDepth;

    double left_bf;
    double front_bf;
    double right_bf;
    double back_bf;

    cv::Mat K;
    cv::Mat distCoef;

//private:
    double left_cxL;
    double left_cxR;

    double front_cxL;
    double front_cxR;

    double right_cxL;
    double right_cxR;

    double back_cxL;
    double back_cxR;
};
