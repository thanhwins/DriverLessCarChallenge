#ifndef SIGNDETECTION_H
#define SIGNDETECTION_H
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <cstdio>

#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


namespace SignDetection {
    class Detection {
        
        public:
            Detection();
            ~Detection();
            void BGR2HSV(double b, double g, double f, double &h, double &s, double &v);
            void BGR2HSV(const cv::Mat &bgr, cv::Mat &hsv);
            bool inside(int u, int v, int n, int m);
            void findComponents(const cv::Mat &binary, std::vector<cv::Rect> &boxes, double lower_ratio=0.5, double upper_ratio=2.0, int lower_numpoint=70);
            void redToBinary(cv::Mat &result, const cv::Mat &bgr);
            void blueToBinary(cv::Mat &result, const cv::Mat &bgr);
            //bool objectLabeling(std::vector<std::vector<cv::Point> > &regs, cv::Mat &depth, ushort &low_th, ushort &high_th);
            bool objectLabeling(std::vector<cv::Rect> &boxes, std::vector<int> &labels, cv::Mat &depth, cv::Mat &color, ushort &low_th, ushort &high_th, int min_pts, int max_pts, ushort min_w, ushort max_w, double ratio);
            //bool objectLabeling(std::vector<cv::Rect> &boxes, std::vector<int> &labels, cv::Mat &depth, cv::Mat &color, ushort &low_th, ushort &high_th, int min_pts = 1000, int max_pts = 8000, ushort min_w = 50, ushort max_w = 120, double ratio = 1.5);
            bool saveDetectedObjects(std::vector<std::vector<cv::Point> > &regs, cv::Mat &image, std::string &path, size_t &image_index);
        private:
            bool floodFill(std::vector<cv::Point> &res, cv::Mat &image, cv::Point &p, ushort connected_condition);
    };
}

#endif // SIGNDETECTION_H
