#include "multilane.h"
#include "api_kinect_cv.h"
// api_kinect_cv.h: manipulate openNI2, kinect, depthMap and object detection
#include <iostream>

using namespace openni;

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define VIDEO_FRAME_WIDTH 320
#define VIDEO_FRAME_HEIGHT 240

char analyzeFrame(const VideoFrameRef& frame_depth,const VideoFrameRef& frame_color,Mat& depth_img, Mat& color_img) {
    DepthPixel* depth_img_data;
    RGB888Pixel* color_img_data;

    int w = frame_color.getWidth();
    int h = frame_color.getHeight();

    depth_img = Mat(h, w, CV_16U);
    color_img = Mat(h, w, CV_8UC3);
    Mat depth_img_8u;
	
   // switch (frame.getVideoMode().getPixelFormat())
  //  {
   //     case PIXEL_FORMAT_DEPTH_1_MM: return 'm';
    //    case PIXEL_FORMAT_DEPTH_100_UM:

            depth_img_data = (DepthPixel*)frame_depth.getData();

            memcpy(depth_img.data, depth_img_data, h*w*sizeof(DepthPixel));

            normalize(depth_img, depth_img_8u, 255, 0, NORM_MINMAX);

            depth_img_8u.convertTo(depth_img_8u, CV_8U);

       //     return 'd';
	
    //    case PIXEL_FORMAT_RGB888:
            color_img_data = (RGB888Pixel*)frame_color.getData();

            memcpy(color_img.data, color_img_data, h*w*sizeof(RGB888Pixel));

            cvtColor(color_img, color_img, COLOR_RGB2BGR);
		
            return 'c';
    //    default:
     //       printf("Unknown format\n");
      //      return 'u';
    //}
}
int main(int argc, char **argv) {
    Status rc;
    Device device;
bool running = false;
char key = 0;
    VideoStream depth, color;
    rc = OpenNI::initialize();
    if (rc != STATUS_OK) {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 0;
    }
    rc = device.open(ANY_DEVICE);
	device.setDepthColorSyncEnabled(true);
    if (rc != STATUS_OK) {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return 0;
    }
    if (device.getSensorInfo(SENSOR_DEPTH) != NULL) {
        rc = depth.create(device, SENSOR_DEPTH);
        if (rc == STATUS_OK) {
            VideoMode depth_mode = depth.getVideoMode();
            depth_mode.setFps(30);
            depth_mode.setResolution(VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT);
            depth_mode.setPixelFormat(PIXEL_FORMAT_DEPTH_100_UM);
            depth.setVideoMode(depth_mode);

            rc = depth.start();
            if (rc != STATUS_OK) {
                printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
            }
        }
        else {
            printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
        }
    }

    if (device.getSensorInfo(SENSOR_COLOR) != NULL) {
        rc = color.create(device, SENSOR_COLOR);
        if (rc == STATUS_OK) {
            VideoMode color_mode = color.getVideoMode();
            color_mode.setFps(30);
            color_mode.setResolution(VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT);
            color_mode.setPixelFormat(PIXEL_FORMAT_RGB888);
            color.setVideoMode(color_mode);

            rc = color.start();
            if (rc != STATUS_OK)
            {
                printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
            }
        }
        else {
            printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
        }
    }
    
    VideoFrameRef frame_depth, frame_color;
    VideoStream* streams[] = {&depth, &color};
/// End of openNI init phase ///
    
/// Init video writer and log files ///   
   
	
	Mat depthImg, colorImg, grayImage;
	int codec = CV_FOURCC('D','I','V', 'X');
	int video_frame_width = VIDEO_FRAME_WIDTH;
    int video_frame_height = VIDEO_FRAME_HEIGHT;
	Size output_size(video_frame_width, video_frame_height);
  
    int frame_width = VIDEO_FRAME_WIDTH;
    int frame_height = VIDEO_FRAME_HEIGHT;
   
   
 

    double st = 0, et = 0, fps = 0;
    double freq = getTickFrequency();


    bool is_show_cam = false;
    while ( true )
    {
       

        st = getTickCount();
       

        ////////  Get input image from 
		int readyStream = -1;
		rc = OpenNI::waitForAnyStream(streams, 2, &readyStream, SAMPLE_READ_WAIT_TIMEOUT);
		if (rc != STATUS_OK){
		    printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
		   break;
		}

		
		depth.readFrame(&frame_depth);	 
		color.readFrame(&frame_color);
		       
		char recordStatus = analyzeFrame(frame_depth,frame_color, depthImg, colorImg);
		flip(depthImg, depthImg, 1);
		flip(colorImg, colorImg, 1);
        cv::cvtColor(colorImg, colorImg, CV_BGR2GRAY);
        cv::Mat dst = keepLanes(colorImg, true);
        cv::imshow("dst", dst);
        cv::Mat two = twoRightMostLanes(dst.size(), dst);
        cv::imshow("two", two);
        cv::waitKey(1);
        return 0;
    }
}
