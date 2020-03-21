#include "opencv2/opencv.hpp"
#include <cstdio>
#include <vector>
#include <queue>
#include <cmath>
#include "../lane_detection/api_lane_detection.h"

#define ksize 110

using namespace cv;
using namespace std;

namespace radon
{
	void radon(const Mat& image, Mat& image_radon, int theta, int edge);
	void radon(const Mat& image, Mat& image_radon, int theta, int edge, bool scale);
	void rsignature(const Mat& radon_img, Mat& rsignature);
	double distance(const Mat& r1, const Mat& r2);
}
