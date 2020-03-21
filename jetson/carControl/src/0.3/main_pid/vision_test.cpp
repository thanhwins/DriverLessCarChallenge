#include "Radon.h"
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
#include <opencv2/objdetect/objdetect.hpp>

using namespace openni;
using namespace cv;
using namespace std;
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

#define MAX_SPEED    60
#define MIN_SPEED        45

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define VIDEO_FRAME_WIDTH_1 640
#define VIDEO_FRAME_HEIGHT_1 480


bool is_save_file = false;
bool is_save_video = true;
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
/** Global variables */
//-- Note, either copy these two files from opencv/data/haarscascades to your current folder, or change these locations
String face_cascade_name = "cascade.xml";
CascadeClassifier cascade;
string window_name = "Capture - Face detection";
RNG rng(12345);
cv::Mat tmp[6];
cv::Mat r[6], r_candidate;
cv::Mat r_sig[6];
cv::Mat r_sig_c;
int countLeft = 0, countRight = 0;
Point getpoint(Point p = Point(0, 0)) {
	lock_guard<mutex> lock(m1);
	cout << "point:  " << p.x << " " << p.y << endl;
    if (is_save_file) {
        fprintf(thetaLogFile, "point: [%d, %d]\n", p.x, p.y);
    }
	if (p == Point(0, 0)) {
		return center_point;
	}
	else {
		center_point = p;
		return Point(0, 0);
	}
}
void compute_signLeftRight() {
	tmp[0] = cv::imread("9.jpg");
	tmp[1] = cv::imread("10.jpg");
	double step = 1, range = 180;
	Radon::Instance()->initRotateMatrixLookupTable(tmp[0].cols, tmp[0].rows);

	for (int i = 0; i < 2; i++) {
		Radon::Instance()->computeRadon(r[i], tmp[i]);

		r_sig[i] = cv::Mat::zeros(1, 180, CV_32FC1);
		Radon::Instance()->computeRSignature(r_sig[i], r[i]);
		//        std::cout << r_sig[i] << std::endl;
	}
}

int test_radon(Mat candidate) {
	Radon::Instance()->computeRadon(r_candidate, candidate);
	r_sig_c = cv::Mat::zeros(1, 180, CV_32FC1);
	Radon::Instance()->computeRSignature(r_sig_c, r_candidate);
	double distanceLeft = 0.0;
	double distanceRight = 0.0;
	for (int i = 0; i < 1; i++) {
		Radon::Instance()->computeDistance(distanceLeft, r_sig[i], r_sig_c);
		std::cout << distanceLeft << " ";
	}
	for (int i = 1; i < 2; i++) {
		Radon::Instance()->computeDistance(distanceRight, r_sig[i], r_sig_c);
		std::cout << distanceRight << " ";
	}
	if (distanceLeft < distanceRight) {
		return 1;
	}
	else if (distanceLeft > distanceRight) {
		return 2;
	}
	std::cout << endl;
	return 0;
}
int detectAndDisplay(Mat &frame)
{
	std::vector<Rect> faces;
	//Mat frame_gray;

	//cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
	//equalizeHist(frame_gray, frame_gray);
    
	cascade.detectMultiScale(frame, faces, 1.1, 3, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));
	if(faces.size() >= 1){
		for (size_t i = 0; i < faces.size(); i++){
			//Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);
			Mat rs;
			cv::Rect roi_left = cv::Rect(faces[i].x, faces[i].y,
				faces[i].width, faces[i].height);
			resize(frame(roi_left).clone(), rs, Size(96, 96));
			//test_radon(rs);
			int idSign = test_radon(rs);
			rectangle(frame, Point(faces[i].x, faces[i].y), Point(faces[i].x + faces[i].width, faces[i].y + faces[i].height), Scalar(0, 255, 0), 3, 8, 0);
			if (idSign == 1) {
				std::cout << "Bien re trai" << std::endl;
				cv::putText(frame, "turn_left", cv::Point(faces[i].x, faces[i].y + faces[i].height + 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 4, 8);
                return 1;
			}
			else if (idSign == 2) {
				cout << "Bien re phai" << endl;
				cv::putText(frame, "turn_right", cv::Point(faces[i].x, faces[i].y + faces[i].height + 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255), 4, 8);
                return 2;
			}
			else {
				cout << "Khong phai" << endl;
			}
		}
		
	}
    return 0;
}
void get_lines_byme(Mat &imgGray, Rect roi, string &tmp_name, vector<Point> &vec_match_loc, int &isLane, int &ymin, int &ymax)
{
	// Blur and threshold
	vec_match_loc.clear();
	Mat img = imgGray(roi).clone();
	GaussianBlur(img, img, Size(15, 15), 0);
	//    Size ksize(9,9);
	//    blur(img, img, ksize );
	Mat img_bin;
	//    threshold( img, img_bin, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
	//adaptiveThreshold(img, img_bin, 255, 0, 0, 71, -11);
	//Canny(img,img_bin,0,220,3);
    img_bin = img.clone();
    for (int i = 0; i < img.rows; i++) {
        for (int j = 0; j < img.cols; j++) {
            if (img.at<uchar>(i, j) >= 200) {
                img_bin.at<uchar>(i, j) = 255;
            } else {
                img_bin.at<uchar>(i, j) = 0;
            }
        }
    }
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
			for (int j = (img_bin.cols - 1); j >= 0; j--) {
				if (img_bin.at<uchar>(i, j) >= 180) {
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
			for (int j = 0; j <= (img_bin.cols - 1); j++) {
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
void centerPointLeft(cv::Mat &imgGray, Point &center_point, bool is_show_output, Point &carPoint, vector<Point> &vec_left_points, vector<Point> &vec_right_points, double &distance) {
    
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
                                 frame_width / 2 - 50, frame_height / 4);
    
    
    ////////////////////////////////////////////////////////////////////
    
    
    string t_name = "";
    int isRight = 0, isLeft = 0;
    int yminL, yminR, ymaxL, ymaxR;
    get_lines_byme(imgGray, roi_left, t_name, vec_left_points, isLeft, yminL, ymaxL);
    
    int betaLeft = 60;
    center_point.x = betaLeft;
    //center_point.x = vec_left_points[0].x + betaLeft;
    center_point.y = vec_left_points[0].y;
    
}
void centerPointRight(cv::Mat &imgGray, Point &center_point, bool is_show_output, Point &carPoint, vector<Point> &vec_left_points, vector<Point> &vec_right_points, double &distance) {
    
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
                                 frame_width / 2 - 50, frame_height / 4);
    cv::Rect roi_right = cv::Rect(roi_left.width, roi_left.y,
                                  frame_width - roi_left.width - 50, roi_left.height);
    
    
    ////////////////////////////////////////////////////////////////////
    
    
    string t_name = "";
    int isRight = 0, isLeft = 0;
    int yminL, yminR, ymaxL, ymaxR;
    get_lines_byme(imgGray, roi_right, t_name, vec_right_points, isRight, yminR, ymaxR);
    
    int betaRight = 80;
    center_point.x = vec_right_points[0].x - betaRight;
    //center_point.x = frame_width - betaRight;
    center_point.y = vec_right_points[0].y;

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
		frame_width / 2 - 50, frame_height / 4);

	cv::Rect roi_right = cv::Rect(roi_left.width, roi_left.y,
		frame_width - roi_left.width - 50, roi_left.height);


	////////////////////////////////////////////////////////////////////


	string t_name = "";
	int isRight = 0, isLeft = 0;
	int yminL, yminR, ymaxL, ymaxR;
	get_lines_byme(imgGray, roi_left, t_name, vec_left_points, isLeft, yminL, ymaxL);

	get_lines_byme(imgGray, roi_right, t_name, vec_right_points, isRight, yminR, ymaxR);
	int nguong = 40;
	int deltaY = 10;
	int alpha = 25;
    int beta = 30;
    std::cout << "isRight : " << isRight << std::endl;
    std::cout << "isLeft : " << isLeft << std::endl;
    std::cout << "count Right   : " << countRight << std::endl;
    std::cout << "count Left : " << countLeft << std::endl;

	if (isRight > 0 && isLeft > 0)
	{
		if (isLeft == 1 && isRight == 2 && abs(ymaxL - yminR) < deltaY) {
			cout << "1" << endl;
            if (is_save_file) {
                fprintf(thetaLogFile, "1\n");
            }
			center_point.x = vec_left_points[0].x + distance / 2 + alpha;
			center_point.y = vec_left_points[0].y;
		}
		else if (isLeft == 2 && isRight == 1 && abs(ymaxL - yminR) < deltaY)
		{
			cout << "2" << endl;
            if (is_save_file) {
			   fprintf(thetaLogFile, "2\n");
            }
			center_point.x = vec_right_points[0].x - distance / 2 - alpha;

			center_point.y = vec_right_points[0].y;
		}
		else
		{
			double newdistance = vec_right_points[0].x - vec_left_points[0].x;
			cout << "3" << endl;
            if (is_save_file) {
                fprintf(thetaLogFile, "3\n");
            }
			if (abs(distance - newdistance) < frame_width / 4) {

				distance = newdistance;
				center_point.x = (vec_left_points[0].x + vec_right_points[0].x) / 2;
				center_point.y = (vec_left_points[0].y + vec_right_points[0].y) / 2;
			}
			//     }

		}

	}
	else if(countLeft == 0){
        if (is_save_file) {
            fprintf(thetaLogFile, "Mat trai ne\n");
        }
		cout << "Mat trai ne" << endl;
		center_point.x = vec_right_points[0].x / 2 - beta;
		center_point.y = vec_right_points[0].y;
	}
	else if(countRight == 0){
        if (is_save_file) {
            fprintf(thetaLogFile, "Mat phai ne\n");
        }
		cout << "Mat phai ne" << endl;
		center_point.x = (vec_left_points[0].x + 640) / 2 ;
		center_point.y = vec_left_points[0].y;
	}
	else if (isRight > 0 && isLeft == 0)
	{
		if (isRight == 1) {
			cout << "5" << endl;
            if (is_save_file) {
                fprintf(thetaLogFile, "5\n");
            }
			if (abs(prvpoint.x - vec_right_points[0].x) < nguong) {
				center_point.x = vec_right_points[0].x - distance / 2 - alpha;
				center_point.y = vec_right_points[0].y;
			}
		}
		else {
			cout << "6" << endl;
            if (is_save_file) {
                fprintf(thetaLogFile, "6\n");
            }
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
            if (is_save_file) {
                fprintf(thetaLogFile, "7\n");
            }
			if (abs(prvpoint.x - vec_right_points[0].x) < nguong) {
				center_point.x = vec_left_points[0].x + distance / 2 + alpha;
				center_point.y = vec_left_points[0].y;
			}
		}
		else if (isLeft == 2) {
			cout << "8" << endl;
            if (is_save_file) {
                fprintf(thetaLogFile, "8\n");
            }
			if (abs(prvpoint.x - vec_right_points[0].x) < nguong) {
				center_point.x = vec_right_points[0].x - distance / 2 + alpha;
				center_point.y = vec_left_points[0].y;
			}
		}

	}
	cout << "Tam trong:" << center_point.x << " " << center_point.y << endl;
    if (is_save_file) {
        fprintf(thetaLogFile, "Tam trong: [%d, %d]\n", center_point.x, center_point.y);
    }
}



int main( int argc, char* argv[] ) {
    compute_signLeftRight();
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
	
    int video_frame_width = VIDEO_FRAME_WIDTH_1;
    int video_frame_height = VIDEO_FRAME_HEIGHT_1;
    
    VideoWriter depth_videoWriter;
    VideoWriter color_videoWriter;
    VideoWriter original_videoWriter;
    
    if (is_save_video == true) {
        
        string original_filename = "original.avi";
        string color_filename = "color.avi";
        string depth_filename = "depth.avi";
        int codec = CV_FOURCC('M', 'J', 'P', 'G');
        Size output_size(video_frame_width, video_frame_height);
        
        color_videoWriter.open(color_filename, codec, 8, output_size, true);
        original_videoWriter.open(original_filename, codec, 8, output_size, true);
    }
	Point centerL(video_frame_width / 4, video_frame_height * 7 / 8);
	Point centerR(video_frame_width * 3 / 4, video_frame_height * 7 / 8);
	Point prvPosition(frame_width / 2 - 14, frame_height * 7 / 8);
	double distanceLane = frame_width / 2;
	carPosition = Point(frame_width / 2 - 14, frame_height);
	int dem = 0;
	int k = 0;
	if(is_save_file == true){
		thetaLogFile = fopen("thetaLog.txt", "w");
	}
	bool is_show_cam = false;
    bool is_sign = true;
    int kSign = 0;
    cascade.load(face_cascade_name);
    int idSign = 0;
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
            Mat colorTemp1 = colorImg.clone();
            
            if (is_sign){
                cout << "detect bien";
                idSign = detectAndDisplay(colorTemp);
                if (idSign > 0) {
                    kSign = k;
                    is_sign = false;
                    throttle_val = MIN_SPEED;
                    api_set_FORWARD_control( pca9685,throttle_val);
                }
            }
            if (k - kSign > 100 && is_sign == false) {
                is_sign = true;
                if(argc == 2 ) throttle_val = atoi(argv[1]);
                api_set_FORWARD_control( pca9685,throttle_val);
            }
            
			if (is_save_video == true) {
                original_videoWriter.write(colorTemp);
            }
			cvtColor(colorTemp, grayImage, CV_BGR2GRAY);
			vector< Point > vec_left_points;


			vector< Point > vec_right_points;
            if (idSign == 1) {
                centerPointLeft(grayImage, tmp_point, is_show_cam, carPosition, vec_left_points, vec_right_points, distanceLane);
            } else if (idSign == 2) {
                centerPointRight(grayImage, tmp_point, is_show_cam, carPosition, vec_left_points, vec_right_points, distanceLane);
            } else {
                centerPoint(grayImage, tmp_point, is_show_cam, carPosition, vec_left_points, vec_right_points, distanceLane);
            }
			if (tmp_point.x == 0 && tmp_point.y == 0 || tmp_point.x <= 0) {
				tmp_point.x = prvPosition.x;
				tmp_point.y = prvPosition.y;	
			}
			//getpoint(tmp_point);
			std::cout << "tam that : " << tmp_point.x << " " << tmp_point.y << std::endl;
			 double angDiff = getTheta(carPosition, tmp_point);
	       if(-10<angDiff&&angDiff<10)angDiff=0;
                theta = (angDiff*2.0);
                std::cout<<"angdiff"<<angDiff<<std::endl;
                std::cout<<"The ta : "<<theta<<std::endl;
		
		       // theta = (0.00);
		        api_set_STEERING_control(pca9685,theta);
            if (is_save_file) {
                fprintf(thetaLogFile, "angdiff : [%lf]\n", angDiff);
                fprintf(thetaLogFile, "theta : [%lf]\n", theta);
            }
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
            if(is_save_video == true){
                cv::circle(colorTemp, tmp_point, 4, cv::Scalar(0, 255, 255), 3);
                color_videoWriter.write(colorTemp);
//                Mat img_bin2 = colorTemp.clone();
//                for (int i = 0; i < colorTemp.rows; i++) {
//                    for (int j = 0; j < colorTemp.cols; j++) {
//                        if (grayImage.at<uchar>(i, j) > 180) {
//                            img_bin2.at<Vec3b>(i, j).val[0] = 255;
//                            img_bin2.at<Vec3b>(i, j).val[1] = 255;
//                            img_bin2.at<Vec3b>(i, j).val[2] = 255;
//                        } else {
//                            img_bin2.at<Vec3b>(i, j).val[0] = 0;
//                            img_bin2.at<Vec3b>(i, j).val[1] = 0;
//                            img_bin2.at<Vec3b>(i, j).val[2] = 0;
//                        }
//                    }
//                }
//                cv::circle(img_bin2, tmp_point, 4, cv::Scalar(0, 255, 255), 3);
//                color_videoWriter.write(img_bin2);
            }
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
    if(is_save_video == true){
        color_videoWriter.release();
        original_videoWriter.release();
    }
    if(is_save_file == true){
        fclose(thetaLogFile);
    }
    throttle_val = 0;
    api_set_FORWARD_control( pca9685, throttle_val);
    std::cout << "Chay den release ne" << std::endl;
		return 0;
}


