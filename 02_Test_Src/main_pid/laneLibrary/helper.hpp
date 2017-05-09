#define TARGET_H

#include <cstdio>
#include <vector>
#include <iostream>
#include <algorithm>
#include "polyfit.h"
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

vector<double> mean(Mat img, int low, int high);
Mat apply_color_mask(Mat img_HSV, Mat warped, Scalar low, Scalar high);
Mat gaussian_blur(Mat warped, int kernel = 5);
Mat abs_sobel_thresh(Mat img, int orientation=0, int sobel_kernel = 3, uchar low = 0, uchar high = 255);
Mat mag_thresh(Mat img, int sobel_kernel = 3, double low = 0, double high = 255);
Mat dir_threshold(Mat img, int sobel_kernel = 3, float low = 0, float high = CV_PI / 2);
Mat sobel_combined(Mat img);
int get_max_mean(std::vector<double> mean, int left, int right);
Mat get_lane_mask(Mat warped);
Mat get_sobel_mask(Mat warped);
Mat combie_sobel_lane(Mat sobel_mask, Mat lane_mask);