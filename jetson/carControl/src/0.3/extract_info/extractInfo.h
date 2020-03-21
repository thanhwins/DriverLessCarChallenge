#include "opencv2/opencv.hpp"
#include <cstdio>
#include <vector>
#include <cmath>
#include "../lane_detection/api_lane_detection.h"

using namespace cv;
using namespace std;

namespace extract
{
	void extract(const Mat& img, string& output, string method);
	void templateMatching(const Mat& img, char &output, const vector<Mat>& templs);
	void radonExtract(const Mat& img, char &output, const Mat& rsignature, int edge);
}
