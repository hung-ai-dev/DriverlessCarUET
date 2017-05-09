/**
    This code runs our car automatically and log video, controller (optional)
    Line detection method: Canny
    Targer point: vanishing point
    Control: pca9685
    
    You should understand this code run to image how we can make this car works from image processing coder's perspective.
    Image processing methods used are very simple, your task is optimize it.
    Besure you set throttle val to 0 before end process. If not, you should stop the car by hand.
    In our experience, if you accidental end the processing and didn't stop the car, you may catch it and switch off the controller physically or run the code again (press up direction button then enter).
**/

// #define DEBUG
#ifdef DEBUG
    #define debug(x) cerr << #x << ": " << x << std::endl;
#else
    #define debug(x)
#endif

#include <sys/time.h>
#include <cstdlib>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "ImgProc/ImgProc.h"
#include "ImgProc/LaneDetector.h"
#include "OpenNI2_Helper.h"
#include "peripheral_driver/i2c/api_i2c_pwm.h"
#include "peripheral_driver/uart/api_uart.h"

using namespace openni;
using namespace cv;

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

#define RAW_WIDTH 320
#define RAW_HEIGHT 240
#define FPS 30

/// Return angle between veritcal line containing car and destination point in degree
double getTheta(Point2f dst) {
    if (dst.x == 0) return 0;
    double ans = atan2(abs(dst.x), dst.y) * 180 / M_PI;
    ans /= 1.7;
    if (dst.x  < 0) ans = -ans;
    return ans;
}

/// Wrapper struct to communicate with arduino in high level
struct SpeedController {
    int portId; // uart port id
    char buf_send[32], buf_read[32];
    SpeedController() {}

    int init() {
        portId = api_uart_open();
        if (portId == -1)
            fprintf(stderr, "Can not open COM port!\n");
        return portId;
    }

    // Since arduino is pretty responsive, there not necessary to set delay
    void setSpeed(int speed) {
        sprintf(buf_send, "f%d\n", speed);
        api_uart_write(portId, buf_send);
    }

    // Read log(Serial.print) from uart, log is stored in buf_read[0..n)
    int readLog() {
        int n = api_uart_read(portId, buf_read);
        #ifdef DEBUG
            for(int i = 0; i < n; ++i) fprintf(stderr, "%c", buf_read[i]);
            fprintf(stderr, "\n");
        #endif
        return n;
    }
};

struct ThetaController {
    const double delayingTime = 0.15;
    double lastSetTime;
    int cnt;
    PCA9685 *pca9685;
    ThetaController() {}

    // Get time of day
    static double wallTime() {
        timeval tv;
        gettimeofday(&tv, NULL);
        return tv.tv_sec + tv.tv_usec * 1e-6;
    }

    void init() {
        pca9685 = new PCA9685();
        lastSetTime = wallTime();
        cnt = 0;
        api_pwm_pca9685_init( pca9685 );
    }

    // Move servo to theta with restrict of delayingTime
    int setTheta(double theta) {
        double curTime = wallTime();
        if (curTime - lastSetTime < delayingTime) {cnt++; return -1;}
        lastSetTime = curTime;
        int pwm2 = api_pwm_set_theta(pca9685, theta);
        // std::cout << cnt << std::endl;
        cnt = 0;
        return pwm2;
    }

    void release() {
        api_pwm_pca9685_release(pca9685);
    }
};


int main( int argc, char* argv[] ) {
/// Init openNI ///
    openni::Device	* p_device;
    openni::VideoStream	depthStream, colorStream;
	openni::VideoFrameRef m_depthFrame, m_colorFrame;
	
	openni::VideoStream* streams[] = { &depthStream, &colorStream };
	setupAstra(p_device, depthStream, colorStream, RAW_WIDTH, RAW_HEIGHT, FPS);

	ImgProc3D::LaneDetector lane_detector(RAW_WIDTH,RAW_HEIGHT);
	lane_detector.setCamera(ImgProc3D::IntrMode_320x240_RAW);
	bool programShouldClose = false;

/// End of openNI init phase //
    double theta = 0.0;
    ThetaController controlTheta;
    SpeedController controlSpeed;
    if (controlSpeed.init() == -1) return 0;
    controlTheta.init();
    
    // Argc == 2 eg ./test-autocar 27 means initial throttle is 27
    int defaultSpeed = 0;
    if(argc == 2) sscanf(argv[1], "%d", &defaultSpeed);
    fprintf(stderr, "Default throttle: %d\n", defaultSpeed);

    bool running = false, started = false, stopped = false;
    double st = 0, et = 0, fps = 0;
    double freq = getTickFrequency();

    while ( true ) {
        Point center_point(0,0);

        st = getTickCount();
        char key = getkey();

        if( key == 's') {
            running = !running;
        }

        if( key == 'f') {
            fprintf(stderr, "End process.\n");
            controlSpeed.setSpeed(0);
            controlTheta.setTheta(0);
            break;
        }

        if( running ) {
			//// Check PCA9685 driver ////////////////////////////////////////////
            if (controlTheta.pca9685->error < 0){
                cerr << "Error: PWM driver"<< endl;
                break;
            }
			if (!started) {
    			fprintf(stderr, "ON\n");
			    started = true; stopped = false;
				controlSpeed.setSpeed(defaultSpeed);
			}

            ////////  Get input image from camera   //////////////////////////////
            int changedStreamDummy;
		    char rc = openni::OpenNI::waitForAnyStream(streams, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
		    if (rc != openni::STATUS_OK) {
			    fprintf(stderr, "Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, openni::OpenNI::getExtendedError());
			    continue;
		    }
		    if (rc != openni::STATUS_OK){
			    fprintf(stderr, "Read failed!\n%s\n", openni::OpenNI::getExtendedError());
			    continue;
		    }

		    int64 st = cv::getTickCount();
		    colorStream.readFrame(&m_colorFrame);
		    depthStream.readFrame(&m_depthFrame);

		    cv::Mat depthMat = cv::Mat(RAW_HEIGHT, RAW_WIDTH, CV_16UC1, (openni::DepthPixel *)m_depthFrame.getData());
		    //depthMat *= 5;

		    cv::Mat bgrMat;
		    cv::Mat colorRaw = cv::Mat(RAW_HEIGHT, RAW_WIDTH, CV_8UC3, (uint8_t *)m_colorFrame.getData());
		    lane_detector.processFrame(colorRaw, depthMat);

		    cv::Mat gray_img;
            // cv::Mat bgr[3];
            // cv::split(lane_detector.laneMap2D, bgr);
            // cv::imshow("blue", bgr[1]);
            cv::cvtColor(lane_detector.laneMap2D, gray_img, cv::COLOR_BGR2GRAY);
            lane_detector.findLane(gray_img);
		    std::vector<cv::Point2f> centers = lane_detector.findLaneCenter(gray_img, 60, 0, 0);

		    // printf("Detect time = %f \n", (cv::getTickCount() - st) / cv::getTickFrequency());

		    cv::Point2f centerP;
		    float angle;
            cv::Point2f realCen;
            int centerSize = centers.size();
            if (centerSize == 0) {
                controlTheta.setTheta(theta);
            } else 
            if (centerSize == 1) {
                realCen.x = (centers[0].x - 256) * 0.5;
                realCen.y = centers[0].y * 0.5;
                theta = getTheta(realCen);
                controlTheta.setTheta(theta);
            } else {
                cv::Point objLeft;
                cv::Point objRight;
                cv::Point objPos = lane_detector.projectToMap2D(depthMat, objLeft, objRight);    
                if (objPos.x >= 0 && (512.0 - objPos.y) * 0.5 <= 150.0) {
                    int objLeftRight = lane_detector.checkObsPosition(lane_detector.left_line, lane_detector.right_line, objPos);
				    if (checkObjOutOfRoad(objLeft, objRight, objLeftRight, centers) == 0) {
                        int newRadius = 35;
                        bool useLeftLane = 0;
                        bool useRightLane = 0;
                        if (objLeftRight == -1) useRightLane = 1; else useLeftLane = 1;
                        std::vector<cv::Point2f> newCenters = lane_detector.findLaneCenter(gray_img, newRadius, useLeftLane, useRightLane);
                        lane_detector.detectLaneCenter(newCenters, centerP);    

                    } else {
                        // Object out of road
                        lane_detector.detectLaneCenter(centers, centerP);
                    }
                } else {
                    // No Object
                    lane_detector.detectLaneCenter(centers, centerP);
                }
                realCen.x = (centerP.x - 256) * 0.5;
                realCen.y = centerP.y * 0.5;
                theta = getTheta(realCen);
                controlTheta.setTheta(theta);
                // et = cv::getTickCount();
                // std::cout << (et - st) / freq << std::endl;
            }
            
            //if (hasCentroid) {
            //    realCen.x = (centerP.x - 256) * 0.5;
            //    realCen.y = centerP.y * 0.5;
            //    debug(realCen);
            //    debug(theta);
            //   theta = getTheta(realCen);
                
            //    controlTheta.setTheta(theta);
            //} else 
            //    controlTheta.setTheta(theta);
            
            #ifdef DEBUG
            
                    for (int i = 0; i < lane_detector.left_line.size(); i++)
                        cv::rectangle(lane_detector.laneMap2D, lane_detector.left_line[i], 
                                cv::Point(lane_detector.left_line[i].x + 11, lane_detector.left_line[i].y + 11),
                                cv::Scalar(150, 0, 0), 2);
		
                for (int i = 0; i < lane_detector.right_line.size(); i++)
                    cv::rectangle(lane_detector.laneMap2D, lane_detector.right_line[i], 
                            cv::Point(lane_detector.right_line[i].x + 11, lane_detector.right_line[i].y + 11),
                            cv::Scalar(250, 100, 0), 2);

                char buffer[25];
                for (int i = 0; i < centers.size(); i++) {
			        cv::circle(lane_detector.laneMap2D, cv::Point2f(centers[i].x, 512 - centers[i].y), 3, cv::Scalar(100, 250, 0), 2);
		        }
                cv::circle(lane_detector.laneMap2D, cv::Point2f(centerP.x, 512 - centerP.y), 3, cv::Scalar(200, 50, 0), 3);
		        sprintf(buffer, "%d %d", (int)realCen.x, (int)realCen.y);
                cv::putText(colorRaw, buffer, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255), 2);
                cv::imshow("LANE", lane_detector.laneMap2D);
                cv::imshow("cl", colorRaw);
                cv::imshow("d", depthMat);
                cv::imshow("t", lane_detector.objectMask);
                cv::waitKey();
            #endif
        }
        else {
            if (!stopped) {
                fprintf(stderr, "OFF\n");
                stopped = true; started = false;
                controlSpeed.setSpeed(0);
                controlTheta.setTheta(0);
			}
        }
    }
    controlTheta.release();
    return 0;
}