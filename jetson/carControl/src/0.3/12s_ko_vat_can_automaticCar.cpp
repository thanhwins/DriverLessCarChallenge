/**
 This code is to test steering coefficient, you can use this code to find suitable coefficients by hand
 Press w or a to increase pwm2, wheels reach no more then it's STEERING_MAX_LEFT
 Press s or d to decrease pwm2, wheesl reach no more then it's STEERING_MAX_RIGHT
 You should set STEERING_NEUTRAL = (STEERING_MAX_LEFT + STEERING_MAX_RIGHT) / 2.
 Controller coefficients are just approximations. When steering wheels reach their limit, it may sometimes work, sometimes not.
 If you run code ./run-straight, the car run 3 meters without error of 3cm, it's acceptable.
 **/

#include "api_kinect_cv.h"
// api_kinect_cv.h: manipulate openNI2, kinect, depthMap and object detection
#include "api_lane_detection.h"
// api_lane_detection.h: manipulate line detection, finding lane center and vanishing point
#include "api_i2c_pwm.h"
#include "api_uart.h"
#include <iostream>
#include <stdio.h>
#include <ctime>
#include <cstring>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "team_lane_detect.h"
#include "basic_digital_processing.h"
#include "pilicon_digital_processing.h"
char * get_sonar = "t3\n";
using namespace std;

int cport_nr;
char buf_send[BUFF_SIZE];
int throttle = 165;
Mat intrinsic;
Mat distCoeffs;
Mat imageUndistorted;
Mat image;
Mat rs;
bool check = true;
/*
 orange 5 50 50 - 15 255 255
 blue 92 0 0 - 124 256 256
 yellow 20 124 123 - 30 256 256
 green 34 50 50 - 80 220 220
 red 0 200 0 - 19 255 255
 */


bool isRecognize = false;
int preX;
Point up(0,0);
Point prePoint;
Point centerPoint;
int space = 300;
int leftX = 300, rightX = 300;
#define MIN_THROTTLE 50;
#define MAX_THROTTLE 70 ;
#define MIN_THROTTLE_LOW 45;
#define MAX_THETA_HIGH 170
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define VIDEO_FRAME_WIDTH 640
#define VIDEO_FRAME_HEIGHT 480
#define MAX_THETA 130
#define OBSTACLE 50
#define OBSTACLE_DETECT 45
#define OBSTACLE_SPACE 50
#define PID 3.5

#define iLowH 20
#define iLowS 124
#define iLowV 123
#define iHighH 30
#define iHighS 256
#define iHighV 256
#define FAR 80
#define NOHUB 1
#define offset 8
Mat hsvImg;
Point vanishPoint(0, 0);

bool detectObstacle = false;
bool isObstacle = false;
struct P1
{
    Mat M, warped;
    
};


/// Get depth Image or BGR image from openNI device
/// Return represent character of each image catched
char analyzeFrame(const VideoFrameRef& frame, Mat& depth_img, Mat& color_img) {
    DepthPixel* depth_img_data;
    RGB888Pixel* color_img_data;
    
    int w = frame.getWidth();
    int h = frame.getHeight();
    
    depth_img = Mat(h, w, CV_16U);
    color_img = Mat(h, w, CV_8UC3);
    Mat depth_img_8u;
    
    switch (frame.getVideoMode().getPixelFormat())
    {
        case PIXEL_FORMAT_DEPTH_1_MM: return 'm';
        case PIXEL_FORMAT_DEPTH_100_UM:
            
            depth_img_data = (DepthPixel*)frame.getData();
            
            memcpy(depth_img.data, depth_img_data, h*w*sizeof(DepthPixel));
            
            normalize(depth_img, depth_img_8u, 255, 0, NORM_MINMAX);
            
            depth_img_8u.convertTo(depth_img_8u, CV_8U);
            
            return 'd';
        case PIXEL_FORMAT_RGB888:
            color_img_data = (RGB888Pixel*)frame.getData();
            
            memcpy(color_img.data, color_img_data, h*w*sizeof(RGB888Pixel));
            
            cvtColor(color_img, color_img, COLOR_RGB2BGR);
            
            return 'c';
        default:
            printf("Unknown format\n");
            return 'u';
    }
}



P1 four_point_transform1(Mat image, Point2f pts[4])
{
    
    Point2f rect[4];
    rect[0].x = rect[0].y = 0.0;
    rect[1].x = rect[1].y = 0.0;
    rect[2].x = rect[2].y = 0.0;
    rect[3].x = rect[3].y = 0.0;
    Point2f diff[4];
    float a[4];
    a[0] = (pts[0].x + pts[0].y);
    a[1] = (pts[1].x + pts[1].y);
    a[2] = (pts[2].x + pts[2].y);
    a[3] = (pts[3].x + pts[3].y);
    
    int t = 0, t1 = 0;
    float mina = a[0], maxa = a[0];
    for (int i = 1; i < 4; i++)
    {
        if (a[i] > mina)
        {
            mina = a[i];
            t = i;
        }
    }
    for (int i = 1; i < 4; i++)
    {
        if (a[i] < maxa)
        {
            maxa = a[i];
            t1 = i;
        }
    }
    
    rect[0] = pts[t1];
    rect[2] = pts[t];
    
    a[0] = (-pts[0].x + pts[0].y);
    a[1] = (-pts[1].x + pts[1].y);
    a[2] = (-pts[2].x + pts[2].y);
    a[3] = (-pts[3].x + pts[3].y);
    
    t = 0, mina = a[0], maxa = a[0], t1 = 0;
    for (int i = 1; i < 4; i++)
    {
        if (a[i] > mina)
        {
            mina = a[i];
            t = i;
        }
    }
    for (int i = 1; i < 4; i++)
    {
        if (a[i] < maxa)
        {
            maxa = a[i];
            t1 = i;
        }
    }
    
    rect[1] = pts[t1];
    rect[3] = pts[t];
    
    
    P1 p;
    
    Point2f tl = rect[0];
    Point2f tr = rect[1];
    Point2f br = rect[2];
    Point2f bl = rect[3];
    
    float widthA = sqrt(((br.x - bl.x) * (br.x - bl.x)) + ((br.y - bl.y) * (br.y - bl.y)));
    float widthB = sqrt(((tr.x - tl.x) * (tr.x - tl.x)) + ((tr.y - tl.y) * (tr.y - tl.y)));
    float maxWidth = max(int(widthA), int(widthB));
    
    float heightA = sqrt(((tr.x - br.x) * (tr.x - br.x)) + ((tr.y - br.y) * (tr.y - br.y)));
    float heightB = sqrt(((tl.x - bl.x) * (tl.x - bl.x)) + ((tl.y - bl.y) * (tl.y - bl.y)));
    float maxHeight = max(int(heightA), int(heightB));
    
    Point2f dst[4] = { { 0, 0 }, { maxWidth - 1, 0 }, { maxWidth - 1, maxHeight - 1 }, { 0, maxHeight - 1 } };
    
    p.M = getPerspectiveTransform(rect, dst);
    warpPerspective(image, p.warped, p.M, Size(maxWidth, maxHeight));
    
    
    return p;
}

void print(Mat mat){
    int heigh = mat.rows;
    int width = mat.cols;
    for(int i = 0; i < heigh; i++){
        for(int j = 0; j < width; j++){
            cout << (int)mat.at<uchar>(i,j) << endl;
        }
        cout << endl;
    }
}
int maxSpace = 0;
void colorDetect(Mat colorImg){
    isObstacle = false;
    detectObstacle = false;
    Mat imgHSV;
    maxSpace = 0;
    Mat color = colorImg;
    cvtColor(color, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV,0), Scalar(iHighH, iHighS, iHighV,0), imgThresholded); //Threshold the image
    //morphological opening (remove small objects from the foreground)
    
    int row = imgThresholded.rows;
    int col = imgThresholded.cols;
    int countR = 0;
    int countL = 0;
    maxSpace  =0;
   	int  mid = vanishPoint.x;
    for(int i = row/3; i < row ; i++){
        for(int j = col; j > 0; j--){
            if(imgThresholded.at<uchar>(i,j) == 255){
                countR++;
                if(maxSpace < j)
                    maxSpace = j;
                break;
            }
        }
        
    }
    if(countR > 50){
        detectObstacle = true;
    }
    if(countR > FAR){
        isObstacle = true;
        cout << "Count " << countR << endl;
        //   isRecognize = true;
    }
    //cvtColor(imgThresholded, imgThresholded, COLOR_GRAY2BGR);
    // return imgThresholded;
}

Point getVanishPoint()
{
    Point vanishPoint;
    Mat colored;
    //image = canBangHistogram(image);
    //image = median_filter(image);
    //image = cropImg(image);
    //image = doSang(image);
    Mat edges;
    cvtColor(image,rs,CV_BGR2GRAY);
    GaussianBlur(rs, rs, Size(7, 7), 1.5,1.5);
    //undistort(image, imageUndistorted, intrinsic, distCoeffs);
    //medianBlur(imageUndistorted, imageUndistorted, 15);
    int width = image.size().width;
    int height = image.size().height;
    
    
    Canny(rs, rs, 0, 220, 3);
    
    Mat after;
    //warpPerspective(rs, rs, p.M, Size(imageUndistorted.size().width , imageUndistorted.size().height),
    //	CV_WARP_INVERSE_MAP | CV_INTER_LINEAR);
    
    /*for (int i = image.rows - image.rows / 4; i < image.rows; i++)
     {
     for (int j = 0; j < image.cols; j++)
     {
     image.at<Vec3b>(i, j) = rs.at<uchar>(i, j);
     }
     }*/
    
    //imshow("both", rs);
    cout <<"-----------------------------" << endl;
    int row = 0;
    int px = 0, py = 0;
    int centerX, centerY;
    int mid = prePoint.x;
    int left = 0,right = 0;
    //cout << "Left: " << leftX << " " << rightX <<endl;
    for (int i = rs.rows-1; i >= rs.rows*1/2; i--){
        int dem = 0;
        int kt = 0;
        Point p[1000];
        
        p[0].x = rs.cols - 1;
        p[0].y = i;
        for (int j = mid; j < rs.cols; j++){
            if (rs.at<uchar>(i, j) == 255){
                p[0].y = i;
                p[0].x = j;
                break;
            }
        }
        p[1].y = i;
        p[1].x = 0;
        for (int j = mid - 1; j > 0; j--){
            if (rs.at<uchar>(i, j) == 255){
                p[1].x = j;
                p[1].y = i;
                break;
            }
        }
        
        if(p[0].x == rs.cols-1 && p[1].x == 0){
            centerX = preX;
        }
        else
            if(p[0].x == rs.cols-1){
                centerX = p[1].x + space;
                if(centerX > rs.cols -1){
                	centerX = rs.cols -1;
                }
                //   if(centerX < rs.cols-1) centerX = rs.cols-space;
            }
            else
                if(p[1].x == 0){
                    //    cout << "Left " << p[0].x << " " << rightX << endl;
                    if(isObstacle){
                        centerX = p[1].x + OBSTACLE_SPACE;
                    }
                    else{
                        centerX = p[0].x - space;
                        if(centerX < 0){
                        	centerX = 0;
                        }
                    }
                    // if(centerX < 0) centerX = space;
                }
                else{
                    if(isObstacle){
                        centerX = p[1].x + OBSTACLE_SPACE;
                    }
                    else{
                        centerX = (p[0].x + p[1].x)/2;
                    }
                }
        left += p[1].x;
        right += p[0].x;
        row++;
        centerY = (p[0].y + p[1].y)/2;
        //  cout << centerX << endl;
        mid = centerX;
        px += centerX;
        py += centerY;
        preX = centerX;
    }
    
    left /= row;
    right /= row;
    cout << "Space: " << right - left << endl;
    px = px / row;
    py = py / row;
    vanishPoint.x = px;
    vanishPoint.y = height *3/4;
    row = 0;
    for (int i = rs.rows-1; i >= rs.rows*1/2; i--){
        int dem = 0;
        int kt = 0;
        Point p[1000];
        
        p[0].x = rs.cols - 1;
        p[0].y = i;
        for (int j = mid; j < rs.cols; j++){
            if (rs.at<uchar>(i, j) == 255){
                p[0].y = i;
                p[0].x = j;
                break;
            }
        }
        p[1].y = i;
        p[1].x = 0;
        for (int j = mid - 1; j > 0; j--){
            if (rs.at<uchar>(i, j) == 255){
                p[1].x = j;
                p[1].y = i;
                break;
            }
        }
        if(p[0].x == rs.cols-1 && p[1].x == 0){
            centerX = preX;
        }
        else
            if(p[0].x == rs.cols-1){
                //         cout << "Right " << p[1].x << "  << leftX" <<  endl;
                if(space < (rs.cols - p[1].x)/2)
                centerX = rs.cols - space;
                else{
                    centerX = p[1].x + space;
                }
                if(centerX > rs.cols-1) centerX = rs.cols-space;
            }
            else
                if(p[1].x == 0){
                    //           cout << "Left " << p[0].x << " " << rightX << endl;
                    if(space < p[0].x/2)
                    centerX = space;
                    else centerX = p[1].x + space;
                    if(centerX < 0) centerX = space;
                }
                else
                    centerX = (p[0].x + p[1].x)/2;
        
        row++;
        centerY = (p[0].y + p[1].y)/2;
        //   cout << centerX << endl;
        mid = centerX;
        px += centerX;
        py += centerY;
        preX = centerX;
    }
    
    px = px / row;
    py = py / row;
    up.x = px;
    up.y = py;
    
    
    return  vanishPoint;
}
double getTheta(Point car, Point dst) {
    if (dst.x == car.x) return 0;
    if (dst.y == car.y) return (dst.x < car.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - car.x;
    double tmp;
    if (car.y < dst.y){
        tmp = car.y;
        car.y = dst.y;
        dst.y = tmp;
    }
    double dy = car.y - dst.y; // image coordinates system: car.y > dst.y
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

void setThrottle(int speed) {
    if (speed >= 0) {
        if (speed == 0) {
            sprintf(buf_send, "t1150%d\n", 0);
        }
        speed = 1500 + speed * 5;
        if (speed >= 3000) {
            speed = 3000;
        }
        if (speed < 1500) {
            speed = 1500;
        }
        sprintf(buf_send, "t1%d\n", speed);
        cout << buf_send << "\n";
    }
    else {
        speed = -speed;
        sprintf(buf_send, "b%d\n", speed);
    }
    api_uart_write(cport_nr, buf_send);
}
// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%m-%d-%H%M%S", &tstruct);
    
    return buf;
}
int main(int argc, char** argv)
{
//    time_t timer;
    clock_t t;
    
    double preTheta = -1000;
    int numBoards = 3;
    int numCornersHor = 9;
    int numCornersVer = 6;
    
    int numSquares = numCornersHor * numCornersVer;
    Size board_sz = Size(numCornersHor, numCornersVer);
    
    
    
    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > image_points;
    
    vector<Point2f> corners;
    int successes = 0;
    
    Status rc;
    Device device;
    
    VideoStream depth, color;
    rc = OpenNI::initialize();
    if (rc != STATUS_OK) {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 0;
    }
    rc = device.open(ANY_DEVICE);
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
    
    VideoFrameRef frame;
    VideoStream* streams[] = { &depth, &color };
    /// End of openNI init phase ///
    
    /// Init video writer and log files ///
    bool is_save_file = false; // set is_save_file = true if you want to log video and i2c pwm coeffs.
    VideoWriter depth_videoWriter;
    VideoWriter color_videoWriter;
    VideoWriter gray_videoWriter;
    
    string gray_filename = "" + currentDateTime() + "_gray.avi";
    
    string color_filename = "" + currentDateTime() + "_color.avi";
    string depth_filename = "depth.avi";
    cout << currentDateTime() << "\n";
    Mat depthImg, colorImg, grayImage;
    int codec = CV_FOURCC('D', 'I', 'V', 'X');
    int video_frame_width = VIDEO_FRAME_WIDTH;
    int video_frame_height = VIDEO_FRAME_HEIGHT;
    Size output_size(video_frame_width, video_frame_height);
    
    
    FILE *thetaLogFile; // File creates log of signal send to pwm control
    if (is_save_file) {
        gray_videoWriter.open(gray_filename, codec, 8, output_size, false);
        color_videoWriter.open(color_filename, codec, 8, output_size, true);
        //   depth_videoWriter.open(depth_filename, codec, 8, output_size, false);
        thetaLogFile = fopen("thetaLog.txt", "w");
    }
    /// End of init logs phase ///
    
    int dir = 0, throttle_val = 0;
    double theta = 0;
    int current_state = 0;
    char key = 0;
    
    //=========== Init  =======================================================
    
    ////////  Init PCA9685 driver   ///////////////////////////////////////////
    
    PCA9685 *pca9685 = new PCA9685();
    api_pwm_pca9685_init(pca9685);
    
    if (pca9685->error >= 0)
        api_pwm_set_control(pca9685, dir, throttle_val, theta, current_state);
    
    /////////  Init UART here   ///////////////////////////////////////////////
    
    cport_nr = api_uart_open();
    
    if (cport_nr == -1) {
        cerr << "Error: Canot Open ComPort";
        return -1;
    }
    
    /// Init MSAC vanishing point library
    MSAC msac;
    cv::Rect roi1 = cv::Rect(0, VIDEO_FRAME_HEIGHT * 3 / 4,
                             VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT / 4);
    
    int frame_width = VIDEO_FRAME_WIDTH;
    int frame_height = VIDEO_FRAME_HEIGHT;
    Point carPosition(frame_width / 2, frame_height);
    dir = 0; int pwm2 = 350;
    theta = 0;
    current_state = 0;
    prePoint.x = frame_width / 2;
    prePoint.y = frame_height;
    centerPoint = prePoint;
    bool running = false, started = false, stopped = false;
    
    double st = 0, et = 0, fps = 0;
    double freq = getTickFrequency();
    
    // 27 is the ESC key
    
    cout << endl << "Hit ESC key to exit" << flush;
    
    key = 0;
    
    bool is_show_cam = false;
    
    
    
    sleep(1);
    int k = 0;
    
    theta = 0;
//    throttle_val = 0;
//    if (!stopped) {
//        fprintf(stderr, "OFF\n");
//        stopped = true; started = false;
//        setThrottle(throttle_val);
//        api_uart_write(cport_nr, buf_send);
//    }
//    api_pwm_set_control(pca9685, dir, throttle_val, theta, current_state);
//    sleep(1);
//    key = 's';
    t = clock();
    sleep(1);
    t = clock() - t;
    t = t *1000/CLOCKS_PER_SEC;
    cout << "----t ---   " << t;
    while (true)
    {
        k++;
        if (k % 3 != 0) continue;
        st = getTickCount();
        key = getkey();
        cout <<"key " << key << endl;
        if (key == 's') {
            running = !running;
            char m2[5];
//            sprintf(buf_send, "m%d\n", 2);
//            api_uart_write(cport_nr, buf_send);
//            key = '@';
        }
        if (key == 'f') {
            fprintf(stderr, "End process.\n");
            theta = 0;
            throttle_val = 0;
            setThrottle(throttle_val);
            api_pwm_set_control(pca9685, dir, throttle_val, theta, current_state);
            break;
        }
        
        if (running)
        {
            //// Check PCA9685 driver ////////////////////////////////////////////
            if (pca9685->error < 0)
            {
                cout << endl << "Error: PWM driver" << endl << flush;
                break;
            }
            if (!started)
            {
                while(1) {
//                    char temp[100];
                    char temp_r[100];
//                    sprintf(temp, "t%d\n", 3);
                    api_uart_write(cport_nr, get_sonar);
                    sleep(0.5);
//                    api_uart_read(cport_nr, temp_r);
                    memset(temp_r, 0, 100);
                    api_uart_read(cport_nr, temp_r);
                    printf("-----sonar string: %s ----", temp_r);
                    int sonar_val = 2;
                    sscanf(temp_r, "%d", &sonar_val);
                    printf("------SONAR: %d ----\n", sonar_val);
                    if (sonar_val > 500 /*|| sonar_val == 1*/) {
                        break;
                    }
                }
                fprintf(stderr, "ON\n");
                started = true; stopped = false;
                throttle_val = throttle;
                setThrottle(throttle_val);
                api_uart_write(cport_nr, buf_send);
            }
            
            
            ////////  Get input image from camera   ////////////////sss//////////////
            int readyStream = -1;
            rc = OpenNI::waitForAnyStream(streams, 2, &readyStream, SAMPLE_READ_WAIT_TIMEOUT);
            if (rc != STATUS_OK)
            {
                printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
                break;
            }
            
            switch (readyStream)
            {
                case 0:
                    // Depth
                    depth.readFrame(&frame);
                    break;
                case 1:
                    // Color
                    color.readFrame(&frame);
                    break;
                default:
                    printf("Unxpected stream\n");
            }
            char recordStatus = analyzeFrame(frame, depthImg, colorImg);
            if(recordStatus == 'd'){
                // print(depthImg);
                //  cout << "Depth" << endl;
            }
            ////////// Detect Center Point ////////////////////////////////////
            if (recordStatus == 'c') {
                /**
                 Status = BGR Image. Since we developed this project to run a sample, we only used information of gray image.
                 **/
                isObstacle = false;
                image = colorImg;
                colorDetect(image);
                vanishPoint = getVanishPoint();
                if (vanishPoint.x == 0){
                    vanishPoint = prePoint;
                }
                if(check == true){
                    int distance = vanishPoint.x -  prePoint.x;
                    if(distance > 150){
                        //   vanishPoint.x = prePoint.x;
                    }
                    if(abs(distance) < 100){
                        // vanishPoint = prePoint;
                    }
                    
                    vanishPoint.x = prePoint.x;
                    check = false;
                }
                bool slow = false;
                
                //if(abs(vanishPoint.x - prePoint.x) > 100) vanishPoint.x = prePoint.x;
                //cout << "x: " << vanishPoint.x << " y: " << vanishPoint.y << "\n";
                
                int px = vanishPoint.x;
                int py = vanishPoint.y;
                
                int tmp = py;
                py = px;
                px = tmp;
                for (int i = px; i < px + 15 && i < frame_width; i++)
                {
                    image.at<Vec3b>(i, py).val[0] = 0;
                    image.at<Vec3b>(i, py).val[1] = 0;
                    image.at<Vec3b>(i, py).val[2] = 255;
                }
                
                for (int i = px; i > px - 15 && i >= 0; i--)
                {
                    image.at<Vec3b>(i, py).val[0] = 0;
                    image.at<Vec3b>(i, py).val[1] = 0;
                    image.at<Vec3b>(i, py).val[2] = 255;
                }
                for (int i = py; i > py - 15 && i >= 0; i--)
                {
                    image.at<Vec3b>(px, i).val[0] = 0;
                    image.at<Vec3b>(px, i).val[1] = 0;
                    image.at<Vec3b>(px, i).val[2] = 255;
                }
                for (int i = py; i < py + 15 && i < image.size().height; i++)
                {
                    image.at<Vec3b>(px, i).val[0] = 0;
                    image.at<Vec3b>(px, i).val[1] = 0;
                    image.at<Vec3b>(px, i).val[2] = 255;
                }
                //                if(isObstacle && vanishPoint.x < maxSpace + OBSTACLE){
                //                    	vanishPoint.x = maxSpace + OBSTACLE_SPACE;
                //                    if(vanishPoint.x > 600){
                //                    	vanishPoint.x = 600;
                //                    }
                //                }
                // TÌnh gÛc quay
                double theta = getTheta(carPosition, vanishPoint);
                prePoint = vanishPoint;
                theta = (-theta * PID);
                // Control car here
                cout << "theta: " << theta << "\n";
                //                pca9685->setPWM(STEERING_CHANNEL2,0, 350 - preTheta + theta);
                //theta -= 8;
                //                if(abs(theta - preTheta) > MAX_THETA){
                //                    slow = true;
                //                }
                //theta -= preTheta;
                //theta = -theta;
                
                if(preTheta == -1000) preTheta = theta;
                if(abs(theta) < MAX_THETA){
                  //  theta = 0;
                }else{
                    if(abs(theta) > 150)
                    slow = true;
                }
                double delta_theta = (theta -preTheta)/5;
                if(abs(theta-preTheta) < 50){
                    //theta = preTheta;
                }
                //setThrottle(0);
                if(slow){
                    throttle_val = MIN_THROTTLE;
                }
                
                else{
                    throttle_val = MAX_THROTTLE;
                }
                if(abs(theta) > MAX_THETA_HIGH){
                	throttle_val = MIN_THROTTLE_LOW;
                }
                if(isObstacle){
                    cout << "Obstacle" << endl;
                    throttle_val = OBSTACLE;
                    //  isObstacle = false;
                }
                if(detectObstacle){
                    throttle_val = OBSTACLE_DETECT;
                }
                cout << throttle_val << endl;
                double new_thetha = theta  - offset;
                api_pwm_set_control(pca9685, dir, throttle_val, new_thetha, current_state);
                preTheta = theta;
                setThrottle(throttle_val);
            }
            ///////  Your PID code here  //////////////////////////////////////////
            if (recordStatus == 'c' && is_save_file) {
                // 'Center': target point
                // pwm2: STEERING coefficient that pwm at channel 2 (our steering wheel's channel)
                //                fprintf(thetaLogFile, "Center: [%d, %d]\n", center_point.x, center_point.y);
                fprintf(thetaLogFile, "pwm2: %d\n", pwm2);
                cv::circle(image, vanishPoint, 4, cv::Scalar(0, 255, 255), 3);
                if (!colorImg.empty())
                    color_videoWriter.write(colorImg);
                if (!image.empty())
                    gray_videoWriter.write(rs);
            }
            if (recordStatus == 'd' && is_save_file) {
                // if (!depthImg.empty())
                //   depth_videoWriter.write(depthImg);
            }
            
            //////// using for debug stuff  ///////////////////////////////////
            if (is_show_cam) {
                if (!image.empty())
                    imshow("color", image);
                waitKey(10);
            }
            if (key == 27) break;
        }
        else {
            theta = 0;
            throttle_val = 0;
            if (!stopped) {
                fprintf(stderr, "OFF\n");
                stopped = true; started = false;
                setThrottle(throttle_val);
                api_uart_write(cport_nr, buf_send);
            }
            api_pwm_set_control(pca9685, dir, throttle_val, theta, current_state);
            sleep(1);
        }
    }
    
    //////////  Release //////////////////////////////////////////////////////
    if (is_save_file)
    {
        gray_videoWriter.release();
        color_videoWriter.release();
        //   depth_videoWriter.release();
        fclose(thetaLogFile);
    }
    return 0;
}






