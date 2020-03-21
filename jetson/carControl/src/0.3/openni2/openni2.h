#ifndef OPENNI2_H
#define OPENNI2_H

#include <iostream>
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "Singleton.h"

namespace framework {
class OpenNI2 : public Singleton<OpenNI2>
{
    friend class Singleton<OpenNI2>;
public:
    OpenNI2();
    virtual ~OpenNI2();
    bool init(bool is_colorize_disp=true, bool is_fixed_max_disp=true, int image_mode=0, bool *retrived_img_flags=NULL);
    bool getDepthMap(cv::Mat &depth);
    bool getValidDepthMap(cv::Mat &valid_depth);
    bool getDisparityMap(cv::Mat &disparity);
    bool getBGRImage(cv::Mat &img);
    bool getGrayImage(cv::Mat &gray);
    bool getIRImage(cv::Mat &ir);
    bool getImage(cv::Mat &img, int type);
    bool getData(cv::Mat &color, cv::Mat &depth, cv::Mat &gray, cv::Mat &disparity);
private:
    bool is_colorize_disp;
    bool is_fixed_max_disp;
    int image_mode;
    cv::VideoCapture capture;
};
}


#endif // OPENNI2_H
