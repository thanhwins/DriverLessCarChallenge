#ifndef MULTILANE_H
#define MUlTILANE_H

#include <vector>
#include <cstdio>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

cv::Mat keepLanes(const cv::Mat &org, bool verbose);
cv::Mat twoRightMostLanes(const cv::Size &size, const cv::Mat &imgLane, cv::Point shift = cv::Point(0, 0), bool right = true);

#endif // MULTILANE_H
