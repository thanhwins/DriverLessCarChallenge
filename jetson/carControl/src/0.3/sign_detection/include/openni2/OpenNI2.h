#ifndef OPENNI2_H
#define OPENNI2_H

#ifdef __linux__
#include <sys/stat.h>
#include <sys/types.h>
#endif

#include <iostream>
#include <vector>
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "common/Singleton.h"

namespace framework {

enum ImgExt {
    JPG=1,
    PNG,
    BMP
};

enum ImgType {
    COLOR=1,
    GRAY,
    DEPTH,
    DISPARITY,
    IR
};

class OpenNI2 : public Singleton<OpenNI2>
{
    friend class Singleton<OpenNI2>;
public:
    OpenNI2();
    virtual ~OpenNI2();
    bool init(bool is_colorize_disp=false, bool is_fixed_max_disp=false, int image_mode=-1, bool *retrived_img_flags=NULL);
    bool getDepthMap(cv::Mat &depth);
    bool getValidDepthMap(cv::Mat &valid_depth);
    bool getDisparityMap(cv::Mat &disparity);
    bool getBGRImage(cv::Mat &img);
    bool getGrayImage(cv::Mat &gray);
    bool getIRImage(cv::Mat &ir);
    bool getImage(cv::Mat &img, int type);
    bool saveData(std::string &path, cv::Mat &img, ImgType type, ImgExt ext, int &index);
    bool colorizeDisparity( const cv::Mat& gray, cv::Mat& rgb, double maxDisp=-1.f, float S=1.f, float V=1.f );


    bool holeLabeling(cv::Mat &disparity, std::vector<std::vector<cv::Point> > &regs);
    bool drawLabeledHole(std::vector<std::vector<cv::Point> > &regs, cv::Mat &image);
    bool depthToBin(cv::Mat &res, cv::Mat &depth, double &low_th, double &high_th);
    bool objectLabeling(std::vector<std::vector<cv::Point> > &regs, cv::Mat &depth, ushort &low_th, ushort &high_th);
    bool saveDetectedObjects(std::vector<std::vector<cv::Point> > &regs, cv::Mat &image, size_t &image_index);
private:
    bool floodFill(std::vector<cv::Point> &res, cv::Mat &image, cv::Point &p);
    bool floodFill(std::vector<cv::Point> &res, cv::Mat &image, cv::Point &p, ushort connected_condition);
private:
    bool is_colorize_disp;
    bool is_fixed_max_disp;
    int image_mode;
    cv::VideoCapture capture;
};
}


#endif // OPENNI2_H
