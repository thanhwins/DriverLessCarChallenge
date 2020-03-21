#include "api_kinect_cv.h"
#include <iostream>
#include <opencv2/imgproc.hpp>
#include "openni2.h"
#include "../openni2/Singleton.h"
#include <unistd.h>
#include "../sign_detection/SignDetection.h"
#include <chrono>
#include "signsRecognizer.h"
#include "extractInfo.h"
#include <stdlib.h>
#include "multilane.h"
#include "Hal.h"
#include "LCDI2C.h"
#include "api_i2c_pwm.h"
#include <iomanip>
#include <queue>
#include <math.h>
#include <thread>
#include <mutex>
#include <condition_variable>
using namespace openni;
using namespace cv;
using namespace framework;
using namespace signDetection;
using namespace EmbeddedFramework;
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#undef debug
#define debug false
#define SW1_PIN	160
#define SW2_PIN	161
#define SW3_PIN	163
#define SW4_PIN	164
#define SENSOR	165
#define LED		166

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define VIDEO_FRAME_WIDTH_1 640
#define VIDEO_FRAME_HEIGHT_1 480
cv::Mat remOutlier(const cv::Mat &gray) {
    int esize = 1;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
    cv::Size( 2*esize + 1, 2*esize+1 ),
    cv::Point( esize, esize ) );
    cv::erode(gray, gray, element);
    std::vector< std::vector<cv::Point> > contours, polygons;
    std::vector< cv::Vec4i > hierarchy;
    cv::findContours(gray, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    for (size_t i = 0; i < contours.size(); ++i) {
        std::vector<cv::Point> p;
        cv::approxPolyDP(cv::Mat(contours[i]), p, 2, true);
        polygons.push_back(p);
    }
    cv::Mat poly = cv::Mat::zeros(gray.size(), CV_8UC3);
    for (size_t i = 0; i < polygons.size(); ++i) {
        cv::Scalar color = cv::Scalar(255, 255, 255);
        cv::drawContours(poly, polygons, i, color, CV_FILLED);
    }
    return poly;
}
double getTheta(Point car, Point dst) {
    if (dst.x == car.x) return 0;
    //if (dst.y == car.y) return (dst.x < car.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - car.x;
    double dy = car.y - dst.y; // image coordinates system: car.y > dst.y
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}
double prv_e=0;
double getTheta_me(Point car,Point dst)
{
    double alpha,theta;
    if (dst.x == car.x) theta = 0;
    //if (dst.y == car.y) theta = (dst.x < car.x ? -90 : 90);
    else
    {
    double L=25,ld=30;
    double pi = acos(-1.0);
    double dx = dst.x - car.x;
    double dy = car.y - dst.y + ld; // image coordinates system: car.y > dst.y
   // if (dx < 0) alpha = -atan(-dx / dy) * 180 / pi;
   // else alpha = atan(dx / dy) * 180 / pi;
    
    if (dx < 0) theta = -atan((2*L/ld)*(-dx /sqrt(dx*dx+dy*dy))) * 180 / pi;
  
    else theta = atan((2*L/ld)*(dx /sqrt(dx*dx+dy*dy))) * 180 / pi;
    //PID
    double kP=0.01,kI=0.02;
   // theta=theta+kP*dx+kI*(dx-prv_e);
    prv_e=dx;
    //
    }
    return theta;
}


///////// utilitie functions  ///////////////////////////
Point carPosition, center_point;
mutex m1;
FILE *thetaLogFile;
int countLeft = 0, countRight = 0;
Point getpoint(Point p = Point(0, 0)) {
	lock_guard<mutex> lock(m1);
	cout << "point:  " << p.x << " " << p.y << endl;
	fprintf(thetaLogFile, "point: [%d, %d]\n", p.x, p.y);
	if (p == Point(0, 0)) {
		return center_point;
	}
	else {
		center_point = p;
		return Point(0, 0);
	}
}

void get_lines_byme(Mat &imgGray, Rect roi, string &tmp_name, vector<Point> &vec_match_loc, int &isLane, int &ymin, int &ymax)
{
	// Blur and threshold
	vec_match_loc.clear();
	Mat img = imgGray(roi).clone();
	GaussianBlur(img, img, Size(11, 11), 0);
	//    Size ksize(9,9);
	//    blur(img, img, ksize );
	Mat img_bin;
	//    threshold( img, img_bin, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	//adaptiveThreshold(img, img_bin, 255, 0, 0, 71, -11);
	Canny(img,img_bin,0,170,3);
	if (roi.x == 0) tmp_name = "left";
	else tmp_name = "right";
	//imshow(tmp_name, img_bin);
	string tmp_nameC = tmp_name + "_contours";
	Mat dst;
	vector<vector<Point> > contours;
	vector<vector<Point> > res_contours;
	Point L(0, 0), R(0, 0);
	countLeft = 0;
	countRight = 0;
	//waitKey(0);
	if (roi.x == 0) {
		int xmin = img_bin.cols, xmax = 0;
		ymin = -1, ymax = -1;

		int t = 0;
		for (int i = img_bin.rows - 2; i >= 0; i--) {
			for (int j = (img_bin.cols - 1) / 2; j >= 0; j--) {
				if (img_bin.at<uchar>(i, j) >= 200) {
					L.x = L.x + (j + roi.x);
					L.y = L.y + (i + roi.y);
					if (xmin > j) {
						xmin = j;
						ymin = i;
					}
					if (xmax < j) {
						xmax = j;
						ymax = i;
					}
					t++;
					countLeft++;
					break;
				
				}
			}
		}
		if (t > 0) {
			L.x = L.x / t;
			L.y = L.y / t;
			if (ymax < ymin && (xmax - xmin) >= 0) isLane = 1;                 //re phai
			else isLane = 2;                          //re trai
		}
		Point minP, maxP;
		minP.x = xmin; maxP.x = xmax; minP.y = ymin; maxP.y = ymax;
		vec_match_loc.push_back(L);
		vec_match_loc.push_back(minP);
		vec_match_loc.push_back(maxP);
	}
	else {
		int t = 0;
		int xmin = img_bin.cols, xmax = 0;
		ymin = -1, ymax = -1;

		for (int i = img_bin.rows - 2; i >= 0; i--) {
			for (int j = 0; j <= (img_bin.cols - 1) / 2; j++) {
				if (img_bin.at<uchar>(i, j) >= 200) {
					R.x += (j + roi.x);
					R.y += (i + roi.y);
					t++;
					if (xmin > j) {
						xmin = j;
						ymin = i;
					}
					if (xmax < j) {
						xmax = j;
						ymax = i;
					}
					countRight++;
					break;
				}
			}
		}
		if (t > 0) {
			R.x = R.x / t;
			R.y = R.y / t;
			if (ymax > ymin && (xmax - xmin) >= 0) isLane = 1;                 //re trai
			else isLane = 2;                          //re phai

		}
		Point minP, maxP;
		minP.x = xmin; maxP.x = xmax; minP.y = ymin; maxP.y = ymax;
		vec_match_loc.push_back(R);
		vec_match_loc.push_back(minP);
		vec_match_loc.push_back(maxP);
	}
}

void centerPoint(cv::Mat &imgGray, Point &center_point, bool is_show_output, Point &carPoint, vector<Point> &vec_left_points, vector<Point> &vec_right_points, double &distance)
{

	vec_left_points.clear();
	vec_right_points.clear();

	//  generate roi
	int frame_width = imgGray.cols;
	int frame_height = imgGray.rows;

	static Point prvpoint(frame_width / 2, frame_height);
	
	Mat dst;
	cv::Rect roi = cv::Rect(0, frame_height * 3 / 8,
		frame_width, frame_height / 2);
	// detect_obs(imgGray, dst,roi);


	cv::Rect roi_left = cv::Rect(0, frame_height * 5 / 8,
		frame_width / 2 - 14, frame_height / 4);

	cv::Rect roi_right = cv::Rect(roi_left.width, roi_left.y,
		frame_width - roi_left.width, roi_left.height);


	////////////////////////////////////////////////////////////////////


	string t_name = "";
	int isRight = 0, isLeft = 0;
	int yminL, yminR, ymaxL, ymaxR;
	get_lines_byme(imgGray, roi_left, t_name, vec_left_points, isLeft, yminL, ymaxL);

	get_lines_byme(imgGray, roi_right, t_name, vec_right_points, isRight, yminR, ymaxR);
	int nguong = 40;
	int deltaY = 10;
	int alpha = 25;
	int beta = 45;
	std::cout << "isRight : " << isRight << std::endl;
	std::cout << "isLeft : " << isLeft << std::endl;

	if (isRight > 0 && isLeft > 0)
	{
		if (isLeft == 1 && isRight == 2 && abs(ymaxL - yminR) < deltaY) {
			cout << "1" << endl;
			fprintf(thetaLogFile, "1\n");
			center_point.x = vec_left_points[0].x + distance / 2 + alpha;
			center_point.y = vec_left_points[0].y;
		}
		else if (isLeft == 2 && isRight == 1 && abs(ymaxL - yminR) < deltaY)
		{
			cout << "2" << endl;
			fprintf(thetaLogFile, "2\n");
			center_point.x = vec_right_points[0].x - distance / 2 - alpha;

			center_point.y = vec_right_points[0].y;
		}
		else
		{
			double newdistance = vec_right_points[0].x - vec_left_points[0].x;
			cout << "3" << endl;
			fprintf(thetaLogFile, "3\n");
			if (abs(distance - newdistance) < frame_width / 4) {

				distance = newdistance;
				center_point.x = (vec_left_points[0].x + vec_right_points[0].x) / 2;
				center_point.y = (vec_left_points[0].y + vec_right_points[0].y) / 2;
			}
			//     }

		}

	}
	else if(countLeft == 0){
		fprintf(thetaLogFile, "Mat trai ne\n");
		cout << "Mat trai ne" << endl;
		center_point.x = vec_right_points[0].x / 2 - beta;
		center_point.y = vec_right_points[0].y;
	}
	else if(countRight == 0){
		fprintf(thetaLogFile, "Mat phai ne\n");
		cout << "Mat phai ne" << endl;
		center_point.x = (vec_left_points[0].x + 640) / 2 ;
		center_point.y = vec_left_points[0].y;
	}
	else if (isRight > 0 && isLeft == 0)
	{
		if (isRight == 1) {
			cout << "5" << endl;
			fprintf(thetaLogFile, "5\n");
			if (abs(prvpoint.x - vec_right_points[0].x) < nguong) {
				center_point.x = vec_right_points[0].x - distance / 2 - alpha;
				center_point.y = vec_right_points[0].y;
			}
		}
		else {
			cout << "6" << endl;
			fprintf(thetaLogFile, "6\n");
			if (abs(prvpoint.x - vec_right_points[0].x) < nguong) {
				center_point.x = vec_left_points[0].x + distance / 2 + alpha;
				center_point.y = vec_right_points[0].y;
			}
		}

	}
	else
	{
		if (isLeft == 1) {
			cout << "7" << endl;
			fprintf(thetaLogFile, "7\n");
			if (abs(prvpoint.x - vec_right_points[0].x) < nguong) {
				center_point.x = vec_left_points[0].x + distance / 2 + alpha;
				center_point.y = vec_left_points[0].y;
			}
		}
		else if (isLeft == 2) {
			cout << "8" << endl;
			fprintf(thetaLogFile, "8\n");
			if (abs(prvpoint.x - vec_right_points[0].x) < nguong) {
				center_point.x = vec_right_points[0].x - distance / 2 + alpha;
				center_point.y = vec_left_points[0].y;
			}
		}

	}
	cout << "Tam trong:" << center_point.x << " " << center_point.y << endl;
	fprintf(thetaLogFile, "Tam trong: [%d, %d]\n", center_point.x, center_point.y);
}



int main( int argc, char* argv[] ) {
	////// init videostream ///
	GPIO *gpio = new GPIO();
	I2C *i2c_device = new I2C();
	LCDI2C *lcd = new LCDI2C();
    int sw1_stat = 1;
	int sw2_stat = 1;
	int sw3_stat = 1;
	int sw4_stat = 1;
	int sensor = 1;
	
	// Setup input
	gpio->gpioExport(SW1_PIN);
	gpio->gpioExport(SW2_PIN);
	gpio->gpioExport(SW3_PIN);
	gpio->gpioExport(SW4_PIN);
	gpio->gpioExport(SENSOR);
	gpio->gpioExport(LED);
	gpio->gpioSetDirection(SW1_PIN, INPUT);
	gpio->gpioSetDirection(SW2_PIN, INPUT);
	gpio->gpioSetDirection(SW3_PIN, INPUT);
	gpio->gpioSetDirection(SW4_PIN, INPUT);
	gpio->gpioSetDirection(SENSOR, INPUT);
	gpio->gpioSetDirection(LED, OUTPUT);
	i2c_device->m_i2c_bus = 2;
	
	if (!i2c_device->HALOpen()) {
		printf("Cannot open I2C peripheral\n");
		exit(-1);
	} else printf("I2C peripheral is opened\n");
	
	unsigned char data;
	if (!i2c_device->HALRead(0x38, 0xFF, 0, &data, "")) {
		printf("LCD is not found!\n");
		exit(-1);
	} else printf ("LCD is connected\n");
	usleep(10000);
	lcd->LCDInit(i2c_device, 0x38, 20, 4);
	lcd->LCDBacklightOn();
	lcd->LCDCursorOn();
	
	lcd->LCDSetCursor(3,1);
	lcd->LCDPrintStr("DRIVERLESS CAR");
	lcd->LCDSetCursor(5,2);
	lcd->LCDPrintStr("2017-2018");
	int dir = 0, throttle_val = 0;
	double theta = 0;
	int current_state = 0;
	char key = 0;

	    //=========== Init  =======================================================
	    ////////  Init PCA9685 driver   ///////////////////////////////////////////
	PCA9685 *pca9685 = new PCA9685() ;
	api_pwm_pca9685_init( pca9685 );
	if (pca9685->error >= 0)api_set_FORWARD_control( pca9685,throttle_val);
	    /// Init MSAC vanishing point library
	MSAC msac;
	api_vanishing_point_init( msac );
	int set_throttle_val = 0;
    throttle_val = 0;
   	theta = 0;
    if(argc == 2 ) set_throttle_val = atoi(argv[1]);
    fprintf(stderr, "Initial throttle: %d\n", set_throttle_val);
    int frame_width = VIDEO_FRAME_WIDTH_1;
    int frame_height = VIDEO_FRAME_HEIGHT_1;
	bool running = false, started = false, stopped = false;
	OpenNI2::Instance() -> init();
	// signsRecognizer recognizer = signsRecognizer("/home/ubuntu/data/new_templates/templates.txt");
	ushort l_th = 600, h_th = 2000;
	vector<vector<Point> > regs;
	Mat depthImg, colorImg, grayImage, disparity;
	VideoWriter depth_videoWriter;
	VideoWriter color_videoWriter;
	VideoWriter original_videoWriter;

	string original_filename = "original.avi";
	string color_filename = "color.avi";
	string depth_filename = "depth.avi";
	int codec = CV_FOURCC('M', 'J', 'P', 'G');
	int video_frame_width = VIDEO_FRAME_WIDTH_1;
	int video_frame_height = VIDEO_FRAME_HEIGHT_1;
	Size output_size(video_frame_width, video_frame_height);
	
	color_videoWriter.open(color_filename, codec, 8, output_size, true);
	original_videoWriter.open(original_filename, codec, 8, output_size, true);
	Point centerL(video_frame_width / 4, video_frame_height * 7 / 8);
	Point centerR(video_frame_width * 3 / 4, video_frame_height * 7 / 8);
	Point prvPosition(frame_width / 2 - 14, frame_height * 7 / 8);
	double distanceLane = frame_width / 2;
	carPosition = Point(frame_width / 2 - 14, frame_height);
	int dem = 0;
	int k = 0;
	bool is_save_file = true;
	if(is_save_file){
		thetaLogFile = fopen("thetaLog.txt", "w");
	}
	bool is_show_cam = false;
    	while ( true )
    	{
    	k++;
		if(k % 3 != 0){
			continue;
		}
		Point tmp_point(0, 0);	
		Point center_point(0,0);
		key = getkey();
       	unsigned int bt_status = 0;
		unsigned int sensor_status = 0;
		gpio->gpioGetValue(SW4_PIN, &bt_status);
		gpio->gpioGetValue(SENSOR, &sensor_status);
		//std::cout<<sensor_status<<std::endl;
		if (!bt_status) {
			if (bt_status != sw4_stat) {
				running = !running;
				sw4_stat = bt_status;
				throttle_val = set_throttle_val;
			}
		} else sw4_stat = bt_status;
	
       if( key == 's') {
            running = !running;
			throttle_val = set_throttle_val;
        }
       	if( key == 'f') {
			fprintf(stderr, "End process.\n");
        	theta = 0;
        	throttle_val = 0;
	    	api_set_FORWARD_control( pca9685,throttle_val);
        	break;
		}
		if( !running )
		{
		lcd->LCDClear();
		lcd->LCDSetCursor(3,1);
		lcd->LCDPrintStr("PAUSE");
		continue;
		}
		if( running ){
		lcd->LCDClear();
		lcd->LCDSetCursor(3,1);
		lcd->LCDPrintStr("RUNNING");
		
		if (!sensor_status) {
			if (sensor_status != sensor) {
				running = !running;
				sensor = sensor_status;
				throttle_val = 0;
			}
		} else sensor = sensor_status;
			if (pca9685->error < 0)
           	 {
                cout<< endl<< "Error: PWM driver"<< endl<< flush;
                break;
           	 }
			if (!started)
			{
    			fprintf(stderr, "ON\n");
			    started = true; stopped = false;
				throttle_val = set_throttle_val;
                api_set_FORWARD_control( pca9685,throttle_val);
			}
        	auto st = chrono::high_resolution_clock::now();
			OpenNI2::Instance()->getData(colorImg, depthImg, grayImage, disparity);
			Mat colorTemp = colorImg.clone();
			original_videoWriter.write(colorTemp);
			cvtColor(colorTemp, grayImage, CV_BGR2GRAY);
			vector< Point > vec_left_points;


			vector< Point > vec_right_points;

			centerPoint(grayImage, tmp_point, is_show_cam, carPosition, vec_left_points, vec_right_points, distanceLane);
			if (tmp_point.x == 0 && tmp_point.y == 0 || tmp_point.x <= 0) {
				tmp_point.x = prvPosition.x;
				tmp_point.y = prvPosition.y;	
			}
			getpoint(tmp_point);
			std::cout << "tam that : " << tmp_point.x << " " << tmp_point.y << std::endl;
			 double angDiff = getTheta(carPosition, tmp_point);
	       if(-10<angDiff&&angDiff<10)angDiff=0;
                theta = (angDiff*2.4);
		std::cout<<"angdiff"<<angDiff<<std::endl;
		std::cout<<"The ta : "<<theta<<std::endl;
		
		       // theta = (0.00);
		        api_set_STEERING_control(pca9685,theta);
		fprintf(thetaLogFile, "angdiff : [%lf]\n", angDiff);
		fprintf(thetaLogFile, "theta : [%lf]\n", theta);
//		        //int pwm2 =  api_set_FORWARD_control( pca9685,throttle_val);
//		//ready = true;
//		//cd.notify_one();
			
//
			if (vec_left_points.size() > 0) {
				for (int i = 0; i < 1; i++) {
					Point p = vec_left_points[i];
					cv::circle(colorTemp, p, 4, cv::Scalar(0, 255, 0), 5);
				}
			}
			if (vec_right_points.size() > 0) {
				for (int i = 0; i < 1; i++) {
					Point p = vec_right_points[i];
					cv::circle(colorTemp, p, 4, cv::Scalar(0, 255, 0), 5);
				}
			}

			cv::circle(colorTemp, tmp_point, 4, cv::Scalar(0, 255, 255), 3);
			color_videoWriter.write(colorTemp);
			prvPosition = tmp_point;
//			color_videoWriter.write(colorImg);
//			auto bt = chrono::high_resolution_clock::now();
//			vector<Rect> boxes;
//			vector<int> labels;
//			cv::Mat pyrDown;
//			SignDetection::Instance()->objectLabeling(boxes, labels, depthImg, colorImg, l_th, h_th, 1000, 8000, 50, 200, 1.5);
//			cv::pyrDown( colorTemp, pyrDown, cv::Size(colorTemp.cols/2, colorTemp.rows/2));
//			cv::Rect roi1 = cv::Rect(0, 240*3/4, 320, 240/4);
//			cvtColor(pyrDown, grayImage, CV_BGR2GRAY);
//            cv::Mat dst = keepLanes(grayImage, false);
//             cv::imshow("dst", dst);
//            cv::Point shift (0, 3 * grayImage.rows / 4);
//        	bool isRight = true;
//		api_get_vanishing_point( grayImage, roi1, msac, center_point, true,"Wavelet");
//            cv::Mat two = twoRightMostLanes(grayImage.size(), dst, shift, isRight);
//            cv::imshow("two", two);
//			Rect roi2(0,   3*two.rows / 4, two.cols, two.rows / 4); //c?t ?nh
//		
//			Mat imgROI2 = two(roi2);
//			 cv::imshow("roi", imgROI2);
//			int widthSrc = imgROI2.cols;
//			int heightSrc = imgROI2.rows;
//			vector<Point> pointList;
//			for (int y = 0; y < heightSrc; y++)
//			{
//				for (int x = widthSrc; x >= 0; x--)
//				{
//					if (imgROI2.at<uchar>(30, x) == 255 )/////////////////25
//					{
//						pointList.push_back(Point(30, x));
//						break;
//					}
//					if(pointList.size() == 0){
//						pointList.push_back(Point(30, 300));
//					}
//				
//				}
//			}
//			std::cout<<"size"<<pointList.size()<<std::endl;
//			int x = 0, y = 0;
//			int xTam = 0, yTam = 0;
//			for (int i = 0; i < pointList.size(); i++)
//				{
//					x = x + pointList.at(i).y;
//					y = y + pointList.at(i).x;
//				}
//			xTam = (x / pointList.size());
//			yTam = (y / pointList.size());
//			xTam = xTam ;
//			if(pointList.size()<=15&&pointList.size()>1)xTam = xTam - 70;
//			yTam = yTam + 240 * 3 / 4;
//			circle(grayImage, Point(xTam, yTam), 2, Scalar(255, 255, 0), 3);
//			 imshow("result", grayImage);
//            if (center_point.x == 0 && center_point.y == 0) center_point = prvPosition;
//            prvPosition = center_point;
//	    center_point = Point(xTam, yTam);
//            double angDiff = getTheta(carPosition, center_point);
//			if(-20<angDiff&&angDiff<20)angDiff=0;
//            theta = (angDiff*2);
//			std::cout<<"angdiff"<<angDiff<<std::endl;
//		        theta = (0.00);
//		    api_set_STEERING_control(pca9685,theta);
//            int pwm2 =  api_set_FORWARD_control( pca9685,throttle_val);
//			auto et = chrono::high_resolution_clock::now();
//		
//			vector<int> signLabels;
//			vector<string> names;
//			 recognizer.labeling(boxes, labels, colorImg, signLabels, names);
//			bt = chrono::high_resolution_clock::now();
//		
//		for (int i = 0; i < 0;i++) //names.size(); i++)
//			{
//				rectangle(colorImg, boxes[i], Scalar(255, 0, 0), 1, 8, 0);
//				if (names[i] == "stop")
//				{
//					cout<<"dungxe";
//					throttle_val = 0;
//					theta = (0.00);	
//					api_set_STEERING_control(pca9685,theta);
//					api_set_FORWARD_control( pca9685,throttle_val);
//					running = !running;
//				}
//				/*if (names[i] == "leftTurn")
//				{
//					cout<<"re trai";
//					theta = (-60.00);	
//					api_set_STEERING_control(pca9685,theta);
//					api_set_FORWARD_control( pca9685,throttle_val);
//					usleep(700000);//running = !running;
//				}*/
//				putText(colorImg, names[i], Point(boxes[i].x, boxes[i].y - 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0));
//				
//			}
//		cout<<"theta"<<theta<<endl;
//		if (debug) printf("Sign_detection run in %.2fms\n", chrono::duration<double, milli> (et-bt).count());
//			 imshow("color", colorImg);
//			
//			char ch = waitKey(10);
//			if (ch == 'q')
//				break;
//			 End Detect traffic signs //////////////
	}
	}
	color_videoWriter.release();
	original_videoWriter.release();
	if(is_save_file){
		fclose(thetaLogFile);
	}
    std::cout << "Chay den release ne" << std::endl;
		return 0;
}


