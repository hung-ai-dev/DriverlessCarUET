#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include "polyfit.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

//find mean value of cols of MAT
vector<double> mean(Mat img, int low, int high) {
    int cols = img.cols;
    vector<double> sum;
    vector<double> res;

    // reduce(img, sum, 1, REDUCE_AVG, REDUCE_SUM );
    // for (int i = 0; i < cols; i+=5) {
    //     double _sum = 0;
    //     _sum = _sum + sum[i] + sum[i + 1] + sum[i + 2] + sum[i + 3] + sum[i + 4];
    //     res.push_back(_sum / (5 * (high - low)));
    // }
    
    int i = 0;
    while (i < cols) {
        double sum = 0;
        for (int j = low; j < high; j++) {
            sum += img.at<uchar>(j, i) + img.at<uchar>(j, i + 1) + img.at<uchar>(j, i + 2)
                    + img.at<uchar>(j, i + 3) + img.at<uchar>(j, i + 4) ;
        }
        res.push_back(sum / (5 * (high - low)));
        i+=5;
    }
    return res;
}

//extract color
Mat apply_color_mask(Mat img_HSV, Mat warped, Scalar low, Scalar high) {
    Mat mask, res;
    inRange(img_HSV, low, high, mask);
    bitwise_and(warped, warped, res, mask);
    return res;
}

//
Mat gaussian_blur(Mat warped, int kernel = 5) {
    Mat res;
    GaussianBlur(warped, res, Size(kernel, kernel), 0, 0);
    return res;
}

//orientation: 0 = x; 1 = y
// sobel thresh
Mat abs_sobel_thresh(Mat img, int orientation=0, int sobel_kernel = 3, uchar low = 0, uchar high = 255) {
    Mat img_s, img_abs;
    int rows = img.rows, cols = img.cols;    
    double min_val, max_val;
    Point2i min_p, max_p;

    if (orientation == 0) 
        Sobel(img, img_s, CV_64F, 1, 0, sobel_kernel); 
    else
        Sobel(img, img_s, CV_64F, 0, 1, sobel_kernel); 
    img_abs = abs(img_s);
   
    minMaxLoc(img_abs, &min_val, &max_val, &min_p, &max_p);

    Mat img_sobel(rows, cols, CV_64F, Scalar(0));    
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++) {
            img_sobel.at<double>(i, j) = 255 * img_abs.at<double>(i, j);
            img_sobel.at<double>(i, j) = img_sobel.at<double>(i, j) / max_val;
        }
    //cout << img_sobel << endl;

    Mat binary_output(rows, cols, CV_8UC1, Scalar(0));
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++) {
            if (img_sobel.at<double>(i, j) >= low && img_sobel.at<double>(i, j) <= high)
                binary_output.at<uchar>(i, j) = 1;
        }
    //cout << binary_output << endl;
    return binary_output;
}

// magnitude thresh
Mat mag_thresh(Mat img, int sobel_kernel = 3, double low = 0, double high = 255) {
    Mat img_sx, img_sy, img_sx2, img_sy2;;
    Mat img_s;
    int rows = img.rows, cols = img.cols;    
    double min_val, max_val;
    Point2i min_p, max_p;

    Sobel(img, img_sx, CV_64F, 1, 0, sobel_kernel); 
    Sobel(img, img_sy, CV_64F, 0, 1, sobel_kernel); 
    pow(img_sx, 2, img_sx2);
    pow(img_sy, 2, img_sy2);
    add(img_sx2, img_sy2, img_sx2);
    sqrt(img_sx2, img_s);

    minMaxLoc(img_s, &min_val, &max_val, &min_p, &max_p);

    Mat img_sobel(rows, cols, CV_64FC1, Scalar(0));
    
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++) {
            img_sobel.at<double>(i, j) = 255 * img_s.at<double>(i, j);
            img_sobel.at<double>(i, j) = img_sobel.at<double>(i, j) / max_val;
        }

    Mat binary_output(rows, cols, CV_8UC1, Scalar(0));
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++) {
            if (img_sobel.at<double>(i, j) >= low && img_sobel.at<double>(i, j) <= high)
                binary_output.at<uchar>(i, j) = 1;
        }
    return binary_output;
}

//direction threshold
Mat dir_threshold(Mat img, int sobel_kernel = 3, float low = 0, float high = CV_PI / 2) {
    Mat img_sx, img_sy;

    Sobel(img, img_sx, CV_64F, 1, 0, sobel_kernel); 
    Sobel(img, img_sy, CV_64F, 0, 1, sobel_kernel); 
    img_sx = abs(img_sx);
    img_sy = abs(img_sy);

    Mat magnitude, angle;

    cartToPolar(img_sx, img_sy, magnitude, angle);

    int rows = img.rows, cols = img.cols;
    Mat binary_output(rows, cols, CV_8UC1, Scalar(0));

    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++) {
            if (angle.at<float>(i, j) >= low && angle.at<float>(i, j) <= high)
                binary_output.at<uchar>(i, j) = 1;
        }
    return binary_output;
}

Mat sobel_combined(Mat img) {
    Mat img_g_mag = mag_thresh(img, 3, 20, 150);
    Mat img_d_mag = dir_threshold(img, 3, 0.6, 1.1);
    Mat img_abs_x = abs_sobel_thresh(img, 0, 5, 50, 200);
    Mat img_abs_y = abs_sobel_thresh(img, 1, 5, 50, 200);

    Mat img_1, img_2, sobel_combined;
    bitwise_and(img_abs_x, img_abs_y, img_1);
    bitwise_and(img_g_mag, img_d_mag, img_2);
    bitwise_or(img_1, img_2, sobel_combined);

    return sobel_combined;
}

int get_max_mean(std::vector<double> mean, int left, int right) {
    int id = 0;
    double max_val = -1;
    double threshold = 20;

    for (int i = left; i < right; i++)
        if (mean[i] > threshold && mean[i] > mean[id]) {
            id = i;
            max_val = mean[i];
        }
    if (max_val == -1)
        return -1;
    return id;
}

// extract white => lane_mask
Mat get_lane_mask(Mat warped) {
    Mat img_HSV;

    cvtColor(warped, img_HSV, COLOR_BGR2HSV);
    Scalar low_white = Scalar(0, 0, 160);
    Scalar high_white =  Scalar(255, 80, 255);
    return apply_color_mask(img_HSV, warped, low_white, high_white); 
}

//operate sobel on L and S channel then combine two mask
Mat get_sobel_mask(Mat warped) {
    Mat img_gauss, img_HLS, img_cmb;
    Mat img_gs[3], warped2, warped3;

    img_gauss = gaussian_blur(warped, 5);
    cvtColor(img_gauss, img_HLS, COLOR_BGR2HLS);
        
    split(img_HLS, img_gs);

    // sobel on L channel
    Mat img_abs_x = abs_sobel_thresh(img_gs[1],'x',5, 40, 225);
    Mat img_abs_y = abs_sobel_thresh(img_gs[1],'y',5, 40, 225);
    Mat img_or;
    bitwise_or(img_abs_x,img_abs_y, img_or);
    warped2 = img_or.clone();

    //sobel on S channel
    img_abs_x = abs_sobel_thresh(img_gs[2],'x',5, 40, 225);
    img_abs_y = abs_sobel_thresh(img_gs[2],'y',5, 40, 225);
    img_or;
    bitwise_or(img_abs_x,img_abs_y, img_or);
    warped3 = img_or.clone();
 
    //combine L and S channel
    bitwise_or(warped2, warped3, img_cmb);

    int rows = img_cmb.rows, cols = img_cmb.cols;
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            img_cmb.at<uchar>(i, j) *= 255;
    return gaussian_blur(img_cmb, 3);
}

//combine lane_mask and sobel mask
Mat combie_sobel_lane(Mat sobel_mask, Mat lane_mask) {
    int rows = sobel_mask.rows, cols = sobel_mask.cols;
    Mat cmb_mask = Mat::zeros(rows, cols, CV_8UC1);

    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++) {
            //cout << (int)lane_mask.at<uchar>(i, j) << " " << (int)img_cmb.at<uchar>(i, j) << endl;
            if (lane_mask.at<uchar>(i, j) >= 140 || sobel_mask.at<uchar>(i, j) >= 127)
                cmb_mask.at<uchar>(i, j) = max(lane_mask.at<uchar>(i, j), sobel_mask.at<uchar>(i, j));
            else
                cmb_mask.at<uchar>(i, j) = 0;
        }
    return cmb_mask;
}
