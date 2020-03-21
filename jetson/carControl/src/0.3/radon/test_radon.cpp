#include "radon.h"

//using namespace radon;
using namespace std;

int main()
{
	Mat src, dst;
	src = imread("/home/ubuntu/data/output/sample_16.png");
	imshow("origin", src);
	waitKey(0);
	radon::radon(src, dst, 180);
//	radon::rsignature(dst, dst);
	imshow("radon", dst);
	waitKey(0);
	return 0;
}
