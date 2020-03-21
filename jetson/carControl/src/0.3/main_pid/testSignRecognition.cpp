#include "SignDetection.h"
#include "SignRecognition.h"
#include "api_kinect_cv.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;
using namespace openni;
int main( int argc, char** argv )
{
SignDetection::Detection detec;
	SignRecognition::SignRecognize a;
double st = 0, et = 0, fps = 0;
    double freq = getTickFrequency();
while ( true )
    {
	st = getTickCount();
    if( argc != 2){
     cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
     return -1;
    }

    Mat image;
    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data ){
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.
	clock_t begin_time = clock();
	int ans = a.recognizeSign(image);
	    	//std::cerr << 1 + (ans - 1) / 5  << '\n';
	cout<< ans << ' ' << float( clock () - begin_time ) /  CLOCKS_PER_SEC << '\n';  	
}                                
    return 0;
}
