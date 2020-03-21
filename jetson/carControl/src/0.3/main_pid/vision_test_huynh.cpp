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
using namespace openni;
using namespace framework;
using namespace signDetection;
using namespace EmbeddedFramework;
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#undef debug
#define debug false
#define SW1_PIN    160
#define SW2_PIN    161
#define SW3_PIN    163
#define SW4_PIN    164
#define SENSOR    165
#define LED        166


int cport_nr;
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
#define VIDEO_FRAME_WIDTH_1 640
#define VIDEO_FRAME_HEIGHT_1 480
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
double getTheta2(Point car, Point dst) {
    if (dst.x == car.x) return 0;
    if (dst.y == car.y) return (dst.x < car.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - car.x;
    double dy = car.y - dst.y; // image coordinates system: car.y > dst.y
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
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
    
    
    //Canny(rs, rs, 0, 200, 3);
    
    Mat after;
    //warpPerspective(rs, rs, p.M, Size(imageUndistorted.size().width , imageUndistorted.size().height),
    //    CV_WARP_INVERSE_MAP | CV_INTER_LINEAR);
    
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

// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
const string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%m-%d-%H%M%S", &tstruct);
    
    return buf;
}
///////// utilitie functions  ///////////////////////////

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
    
    //
    bool is_save_file = true; // set is_save_file = true if you want to log video and i2c pwm coeffs.
    VideoWriter depth_videoWriter;
    VideoWriter color_videoWriter;
    VideoWriter gray_videoWriter;
    
    string gray_filename = "" + currentDateTime() + "_gray.avi";
    
    string color_filename = "" + currentDateTime() + "_color.avi";
    string depth_filename = "depth.avi";
    cout << currentDateTime() << "\n";
    //Mat depthImg, colorImg, grayImage;
    int codec = CV_FOURCC('M', 'J', 'P', 'G');
    int codec2 = CV_FOURCC('D', 'I', 'V', 'X');
    int video_frame_width = VIDEO_FRAME_WIDTH_1;
    int video_frame_height = VIDEO_FRAME_HEIGHT_1;
    Size output_size(video_frame_width, video_frame_height);
    
    
    FILE *thetaLogFile; // File creates log of signal send to pwm control
    if (is_save_file) {
        gray_videoWriter.open(gray_filename, codec, 8, output_size, false);
        color_videoWriter.open(color_filename, codec, 8, output_size, true);
        //   depth_videoWriter.open(depth_filename, codec, 8, output_size, false);
        thetaLogFile = fopen("thetaLog.txt", "w");
    }
    /// End of init logs phase ///
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
    Point carPosition(frame_width / 2, frame_height);
    Point prvPosition = carPosition;
    bool running = false, started = false, stopped = false;
    OpenNI2::Instance() -> init();
    // signsRecognizer recognizer = signsRecognizer("/home/ubuntu/data/new_templates/templates.txt");
    ushort l_th = 600, h_th = 2000;
    vector<vector<Point> > regs;
    Mat depthImg, colorImg, grayImage, disparity;
    while ( true )
    {
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
            //                        vanishPoint.x = maxSpace + OBSTACLE_SPACE;
            //                    if(vanishPoint.x > 600){
            //                        vanishPoint.x = 600;
            //                    }
            //                }
            // TÌnh gÛc quay
            //double theta = getTheta(carPosition, vanishPoint);
            double angDiff = getTheta(carPosition, vanishPoint);
            //if(-20<angDiff&&angDiff<20)angDiff=0;
            theta = (angDiff*2);
            std::cout<<"angdiff"<<angDiff<<std::endl;
            // theta = (0.00);
            api_set_STEERING_control(pca9685,theta);
            int pwm2 =  api_set_FORWARD_control( pca9685,throttle_val);
            auto et = chrono::high_resolution_clock::now();
            if (is_save_file) {
                cv::circle(image, vanishPoint, 4, cv::Scalar(0, 255, 255), 3);
                if (!colorImg.empty())
                    color_videoWriter.write(colorImg);
                if (!image.empty())
                    gray_videoWriter.write(rs);
            }
            vector<int> signLabels;
            vector<string> names;
            // recognizer.labeling(boxes, labels, colorImg, signLabels, names);
            //bt = chrono::high_resolution_clock::now();
            
            
            // imshow("color", colorImg);
            
            char ch = waitKey(10);
            if (ch == 'q')
                break;
            //////// End Detect traffic signs //////////////
        }
    }
    if (is_save_file)
    {
        gray_videoWriter.release();
        color_videoWriter.release();
        //   depth_videoWriter.release();
        fclose(thetaLogFile);
    }
    return 0;
}


