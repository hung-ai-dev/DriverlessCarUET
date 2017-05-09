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
#include "api_lane_detection.h"
// api_lane_detection.h: manipulate line detection, finding lane center and vanishing point
#include "api_i2c_pwm.h"
#include "api_uart.h"
#include <iostream>
#include "laneLibrary/polyfit.h"
#include "laneLibrary/helper.hpp"
#include <algorithm>
#include <vector>
#include "OpenNI.h"
#include "tumrecorder/NISensorController.h"

using namespace openni;

    int kernel_size = 5;
    int n_component = 4, order = 2;
    int part_height, id_peakL, id_peakR, id_peakL_prev, id_peakR_prev, diffR = 0, diffL = 0;
    char key = 0;
    double xData[10], yData[10], coeff[10];
    vector<Point2d> centers;
    vector<double> mean_lane;
    Mat part_image, warp_matrix, reverse_matrix;
    Mat lane_mask, warped, sobel_mask, cmb_mask;

    Mat get_persepective(int width, int height) {
        // The 4-points at the input image  
        vector<Point2f> origPoints;
        origPoints.push_back( Point2f(0, height * 70/ 100));
        origPoints.push_back( Point2f(width, height * 70 / 100));
        origPoints.push_back( Point2f(width, height));
        origPoints.push_back( Point2f(0, height));
        
        // The 4-points correspondences in the destination image
        vector<Point2f> dstPoints;
        dstPoints.push_back( Point2f(0, 0) );
        dstPoints.push_back( Point2f(width, 0) );
        dstPoints.push_back( Point2f(width * 7 / 10, height) );
        dstPoints.push_back( Point2f(width * 3 / 10, height) );

        return getPerspectiveTransform(origPoints, dstPoints);
    }

    Mat reverse_warp_perspective(int width, int height) {
        // The 4-points at the input image  
        vector<Point2f> origPoints;
        origPoints.push_back( Point2f(0, height * 70 / 100));
        origPoints.push_back( Point2f(width, height * 70 / 100));
        origPoints.push_back( Point2f(width, height));
        origPoints.push_back( Point2f(0, height));
        
        // The 4-points correspondences in the destination image
        vector<Point2f> dstPoints;
        dstPoints.push_back( Point2f(0, 0) );
        dstPoints.push_back( Point2f(width, 0) );
        dstPoints.push_back( Point2f(width * 7 / 10, height) );
        dstPoints.push_back( Point2f(width * 3 / 10, height) );

        return getPerspectiveTransform(dstPoints, origPoints);
    }

    Point reverse_point(Mat reverse, Point p) {
        Point dst;
        dst.x = int((reverse.at<double>(0,0) * p.x + reverse.at<double>(0,1) * p.y + reverse.at<double>(0,2)) /
            (reverse.at<double>(2,0) * p.x + reverse.at<double>(2,1) * p.y + reverse.at<double>(2,2)));
        dst.y = int((reverse.at<double>(1,0) * p.x + reverse.at<double>(1,1) * p.y + reverse.at<double>(1,2)) /
            (reverse.at<double>(2,0) * p.x + reverse.at<double>(2,1) * p.y + reverse.at<double>(2,2)));

        return dst;
    }

    Point lane_center(Mat &img_Input, int cols, int rows) {
            // generate combine mask => cmb_mask
                warpPerspective(img_Input, warped, warp_matrix, warped.size(), INTER_LINEAR, BORDER_CONSTANT, 0);
                lane_mask = get_lane_mask(warped);
                sobel_mask = get_sobel_mask(warped);
                cvtColor(lane_mask, lane_mask, COLOR_BGR2GRAY);
                cmb_mask = combie_sobel_lane(sobel_mask, lane_mask);
            // end combine mask

            // caculate peak intensity of lane / 2 (vertical)
                Mat part_image(cmb_mask, Rect(0, int(rows / 2), cols, int(rows / 2)));
                mean_lane.clear();
                mean_lane = mean(part_image, 0, part_image.rows);
                id_peakL = get_max_mean(mean_lane, 0, mean_lane.size() / 2 - 5);
                id_peakR = get_max_mean(mean_lane, mean_lane.size() / 2 + 5, mean_lane.size());

                if (id_peakL == -1)
                    id_peakL = id_peakL_prev;
                else 
                    id_peakL = id_peakL * 5 + 2;

                if (id_peakR == -1)
                    id_peakR = id_peakR_prev;
                else
                    id_peakR = id_peakR * 5 + 2;

                id_peakR_prev = id_peakR;
                id_peakL_prev = id_peakL;
                diffL = 0; diffR = 0;
                // Mat his = Mat::zeros(cmb_mask.rows, cmb_mask.cols, CV_8UC3);
                // for (int i = 1; i < mean_lane.size(); i++) {
                //     line(his,Point(i - 1, his.rows - mean_lane[i - 1]), Point(i, his.rows - mean_lane[i]), Scalar(255, 255, 255), 2);
                // }
            // end calculate

                centers.clear();     
                part_height = cmb_mask.rows / n_component;

            // find centers
                for (int i = 0; i < n_component; i++) {
                    Mat part_image(cmb_mask, Rect(0, part_height*(n_component - i - 1), cols, part_height));

                    mean_lane = mean(part_image, 0, part_image.rows);
                    // Mat his = Mat::zeros(cmb_mask.rows, cmb_mask.cols, CV_8UC3);           
                    // for (int ii = 1; ii < mean_lane.size(); ii++) {
                    //    line(his,Point(ii - 1, his.rows - mean_lane[ii - 1]), Point(ii, his.rows - mean_lane[ii]), Scalar(255, 255, 255), 2);
                    // }
                    // imshow("his", his);

                    id_peakL = get_max_mean(mean_lane, 0, mean_lane.size() / 2 - 5);
                    id_peakR = get_max_mean(mean_lane, mean_lane.size() / 2 + 5, mean_lane.size());

                    if (id_peakL == -1)
                        id_peakL = id_peakL_prev;
                    else 
                        id_peakL = id_peakL * 5 + 2;
                    if (id_peakR == -1)
                        id_peakR = id_peakR_prev;
                    else 
                        id_peakR = id_peakR * 5 + 2;

                    if (abs(id_peakL - id_peakL_prev) >= 110)
                        id_peakL = id_peakL_prev - diffL;
                    if (abs(id_peakR - id_peakR_prev) >= 110)
                        id_peakR = id_peakR_prev - diffR;

                    diffL = id_peakL - id_peakL_prev;
                    diffR = id_peakR - id_peakR_prev;

                    id_peakL_prev = id_peakL;
                    id_peakR_prev = id_peakR;

                    xData[i] = part_height*(n_component - i - 1);
                    yData[i] = (id_peakL + id_peakR) / 2;
                }
            // end find centers
                
            //interpolation center
                int result = polyfit(xData, yData, n_component, order, coeff);
                
                for (int i = 0; i < n_component; i++) {
                    Point center;
                    double sum = 0;

                    for (int ord = 0; ord <= order; ord++) {
                        double tmp = coeff[ord];
                        for (int k = 0; k < ord; k++) {
                            tmp*=xData[i];
                        }
                        sum+=tmp;
                    }            

                    //circle(warped, Point(sum, part_height*(n_component - i - 1)), 5, Scalar(50, 20, 204), 10);
                    Point ori_point = reverse_point(reverse_matrix, Point(sum, part_height*(n_component - i - 1)));
                    centers.push_back(ori_point);
                }
        return centers[3];
    }

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define VIDEO_FRAME_WIDTH 640
#define VIDEO_FRAME_HEIGHT 480

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

/// Return angle between veritcal line containing car and destination point in degree
double getTheta(Point car, Point dst) {
    if (dst.x == car.x) return 0;
    if (dst.y == car.y) return (dst.x < car.x ? -90 : 90);
    double pi = acos(-1.0);
    double dx = dst.x - car.x;
    double dy = car.y - dst.y; // image coordinates system: car.y > dst.y
    if (dx < 0) return -atan(-dx / dy) * 180 / pi;
    return atan(dx / dy) * 180 / pi;
}

int cport_nr; // port id of uart.
char buf_send[BUFF_SIZE]; // buffer to store and recive controller messages.

/// Write speed to buffer
void setThrottle(int speed) {
	if (speed>=0)
    sprintf(buf_send, "f%d\n", speed);
	else { 
		speed=-speed;
		sprintf(buf_send, "b%d\n", speed);
	}
}

///////// utilitie functions  ///////////////////////////

int main( int argc, char* argv[] ) {
    NISensorController ni_sensor;
	ni_sensor.initKinect();
    
/// Init video writer and log files ///   
    bool is_save_file = true; // set is_save_file = true if you want to log video and i2c pwm coeffs.
    VideoWriter depth_videoWriter;	
    VideoWriter color_videoWriter;
    VideoWriter gray_videoWriter;
     
    string gray_filename = "gray.avi";
	string color_filename = "color.avi";
	string depth_filename = "depth.avi";
	
	Mat depthImg, colorImg, grayImage;
	int codec = CV_FOURCC('D','I','V', 'X');
	int video_frame_width = VIDEO_FRAME_WIDTH;
    int video_frame_height = VIDEO_FRAME_HEIGHT;
	Size output_size(video_frame_width, video_frame_height);

   	FILE *thetaLogFile; // File creates log of signal send to pwm control
	if(is_save_file) {
	    gray_videoWriter.open(gray_filename, codec, 8, output_size, false);
        color_videoWriter.open(color_filename, codec, 8, output_size, true);
        //depth_videoWriter.open(depth_filename, codec, 8, output_size, false);
        thetaLogFile = fopen("thetaLog.txt", "w");
	}
/// End of init logs phase ///

    int dir = 0, throttle_val = 0;
    double theta = 0;
    int current_state = 0;
    char key = 0;

    //=========== Init  =======================================================

    ////////  Init PCA9685 driver   ///////////////////////////////////////////

    PCA9685 *pca9685 = new PCA9685() ;
    api_pwm_pca9685_init( pca9685 );

    if (pca9685->error >= 0)
        api_pwm_set_control( pca9685, dir, throttle_val, theta, current_state );

    /////////  Init UART here   ///////////////////////////////////////////////

	cport_nr = api_uart_open();

    if( cport_nr == -1 ) {
        cerr<< "Error: Canot Open ComPort";
        return -1;
    }

    /// Init MSAC vanishing point library
    MSAC msac;
    cv::Rect roi1 = cv::Rect(0, VIDEO_FRAME_HEIGHT*3/4,
                            VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT/4);

    api_vanishing_point_init( msac );

    ////////  Init direction and ESC speed  ///////////////////////////
    dir = DIR_REVERSE;
    int set_throttle_val = 0;
    throttle_val = 0;
    theta = 0;
    
    // Argc == 2 eg ./test-autocar 27 means initial throttle is 27
    if(argc == 2 ) set_throttle_val = atoi(argv[1]);
    fprintf(stderr, "Initial throttle: %d\n", set_throttle_val);
    int frame_width = VIDEO_FRAME_WIDTH;
    int frame_height = VIDEO_FRAME_HEIGHT;
//    warp_matrix = get_persepective(frame_width, frame_height);
//    reverse_matrix = reverse_warp_perspective(frame_width, frame_height);
    Point carPosition(frame_width / 2, frame_height);
    /** 
        We chose car position is center of bottom of the image.
        Indeed, it's a rectangle. We used only a single point to make it simple and enough-to-work.
    **/
    Point prvPosition = carPosition;
    //============ End of Init phase  ==========================================

    //============  PID Car Control start here. You must modify this section ===

    bool running = false, started = false, stopped = false;

    double st = 0, et = 0, fps = 0;
    double freq = getTickFrequency();


    bool is_show_cam = false;
	int count_s,count_ss;
    while ( true )
    {
        cv::waitKey(5);
        Point center_point(0,0);

        st = getTickCount();
        key = getkey();
        if( key == 's') {
            running = !running;
        }
        if( key == 'f') {
            ni_sensor.closeKinect();
            fprintf(stderr, "End process.\n");
            theta = 0;
            throttle_val = 0;
            setThrottle(throttle_val);
            api_uart_write(cport_nr, buf_send);
	        api_pwm_set_control( pca9685, dir, throttle_val, theta, current_state );
            break;
        }

        if( running )
        {
			//// Check PCA9685 driver ////////////////////////////////////////////
            if (pca9685->error < 0)
            {
                cout<< endl<< "Error: PWM driver"<< endl<< flush;
                break;
            }
			if (!started)
			{
    			fprintf(stderr, "ON\n");
			    started = true; stopped = false;
			    ni_sensor.initKinect();
				throttle_val = set_throttle_val;
				setThrottle(throttle_val);
				api_uart_write(cport_nr, buf_send);
			}

            ////////  Get input image from camera   //////////////////////////////
            if (!ni_sensor.hasNewframe) continue;
            ni_sensor.hasNewframe = false;
            cv::Mat colorRaw = cv::Mat(RAW_HEIGHT, RAW_WIDTH, CV_8UC3, ni_sensor.pColor);
			cvtColor(colorRaw, colorImg, CV_RGB2BGR);

            cvtColor(colorImg, grayImage, CV_BGR2GRAY);
            // api_get_lane_center( grayImage, center_point, true);
            api_get_vanishing_point( grayImage, roi1, msac, center_point, is_show_cam,"Wavelet");

            if (center_point.x == 0 && center_point.y == 0) center_point = prvPosition;
            prvPosition = center_point;
            double angDiff = getTheta(carPosition, center_point);
            theta = (-angDiff * 2.0);

            ///////  Your PID code here  //////////////////////////////////////////
			
            int pwm2 = api_pwm_set_control( pca9685, dir, throttle_val, theta, current_state );
            et = getTickCount();
            fps = 1.0 / ((et-st)/freq);
            cerr << "FPS: "<< fps<< '\n';

            if (is_save_file) {
                // 'Center': target point
                // pwm2: STEERING coefficient that pwm at channel 2 (our steering wheel's channel)
                fprintf(thetaLogFile, "Center: [%d, %d]\n", center_point.x, center_point.y);
                fprintf(thetaLogFile, "pwm2: %d\n", pwm2);
                cv::circle(colorImg, center_point, 4, cv::Scalar(0, 255, 255), 3);
                if (!colorImg.empty())
			        color_videoWriter.write(colorImg);
			    if (!grayImage.empty()) {
			        /*int erosion_size = 1;
        cv::Mat element = cv::getStructuringElement( MORPH_RECT,
                                             Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );
                    Mat waveImg;
    			    edgeProcessing(grayImage, waveImg, element, "Wavelet");
    			    imshow("wave", waveImg);
    			    waitKey(10);*/
			        gray_videoWriter.write(grayImage); 
			    }
            }

            //////// using for debug stuff  ///////////////////////////////////
            if(is_show_cam) {
                if(!grayImage.empty())
                    imshow( "gray", grayImage );
                waitKey(10);
            }
            if( key == 27 ) break;
        }
        else {
			theta = 0;
            throttle_val = 0;
            if (!stopped) {
                fprintf(stderr, "OFF\n");
                stopped = true; started = false;
                ni_sensor.closeKinect();
                setThrottle(throttle_val);
                api_uart_write(cport_nr, buf_send);
			}
			api_pwm_set_control( pca9685, dir, throttle_val, theta, current_state );
            sleep(1);
        }
    }
    ni_sensor.closeKinect();
    //////////  Release //////////////////////////////////////////////////////
	if(is_save_file)
    {
        gray_videoWriter.release();
        color_videoWriter.release();
        //depth_videoWriter.release();
        fclose(thetaLogFile);
	}
    return 0;
}


