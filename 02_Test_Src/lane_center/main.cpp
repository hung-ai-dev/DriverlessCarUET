#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include "polyfit.h"
#include "helper.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int kernel_size = 5;
int n_component = 4, order = 2;
int part_height, id_peakL, id_peakR, id_peakL_prev, id_peakR_prev, diffR = 0, diffL = 0;
char key = 0;
double xData[10], yData[10], coeff[10];
vector<Point2d> centers;
vector<double> mean_lane;
Mat part_image, warp_matrix, reverse_matrix;
Mat img_Input, lane_mask, warped, sobel_mask, cmb_mask;

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

Point lane_center(Mat img_Input, int cols, int rows) {
    //named window  
        //namedWindow("warped", WINDOW_NORMAL);
        //namedWindow("sobel", WINDOW_NORMAL);
        //namedWindow("lane_mask", WINDOW_NORMAL);
        // namedWindow("cmb_mask", WINDOW_NORMAL);
        //namedWindow("input", WINDOW_NORMAL);
        // namedWindow("his", WINDOW_NORMAL);
        //namedWindow("win4", WINDOW_NORMAL);
    //end window

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
                // if (i > 0)
                //     line(img_Input, centers[i - 1], centers[i], Scalar(50, 20, 204), 2);
            }

             circle(img_Input, centers[0], 5, Scalar(50, 20, 204), 2);
        imshow("warped", warped);
        imshow("input", img_Input);
    return centers[0];
}

// main main main
int main(int argc, char* argv[]) { 
    VideoCapture video;
    VideoWriter writer;
    if (argc > 1)
        video.open(argv[1]);
    else 
        video.open(0);    

    int width = video.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = video.get(CV_CAP_PROP_FRAME_HEIGHT);

    warp_matrix = get_persepective(width, height);
    reverse_matrix = reverse_warp_perspective(width, height);
    Point center;

    double st, et = 0, fps = 0;
    double freq = getTickFrequency();

    for(int fr = 0; ; fr++) {
        if (fr % 3 != 0)
            continue;
        st = getTickCount();
        video >> img_Input;
        if (img_Input.empty())
            break; 
        center = lane_center(img_Input, width, height);
        et = getTickCount();
        //cout << (et - st) / freq << endl;
        key = waitKey(-1);
        if( key == 27 ) break;
    }
    return 0;
}
