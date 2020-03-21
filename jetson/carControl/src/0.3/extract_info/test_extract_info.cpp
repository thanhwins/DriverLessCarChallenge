#include "extractInfo.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace extract;

int main(int argv, char** args)
{
	Mat img = imread(args[1]);
	string output;
	string method = args[2];
	bool flipImg;
	if (args[3][0] == '0') flipImg = false;
	else flipImg = true;
//	cout << flipImg << " " << args[3] << endl;
	if (flipImg)
	{
		flip(img, img, 1);
	}
	extract::extract(img, output, method);
	cout << output << endl;
}
