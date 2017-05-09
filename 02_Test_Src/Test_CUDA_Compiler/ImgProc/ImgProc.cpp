#include "ImgProc.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#include <opencv2/opencv.hpp>

#define INVALID_CVPOINT2i cv::Point2i(-1, -1)

cv::Point2i getRandomSample(const cv::Mat & depthMat, const cv::Point2i & origin, cv::RNG & rng, const cv::Point2i delta)
{
	cv::Point2i ranPoint;
	int tryCount = 0;
	do {
		ranPoint = cv::Point2i(
			rng.uniform(origin.x - delta.x, origin.x + delta.x),
			rng.uniform(origin.y - delta.y, origin.y + delta.y));
		tryCount++;
		if (tryCount == 100)
		{
			ranPoint = INVALID_CVPOINT2i;
			break;
		}
	} while (depthMat.at<ushort>(ranPoint) == 0);

	return ranPoint;
}

cv::Point3f getPointXYZ(const cv::Mat & depthMat, const ImgProc3D::Intr & camInfo, cv::Point2i p)
{
	float d = depthMat.at<ushort>(p) / camInfo.scale;
	cv::Point3f rs = cv::Point3f(0, 0, 0);
	
	if (d!=0){
		rs = cv::Point3f((p.x - camInfo.cx) * d / camInfo.fx, 
			(p.y-camInfo.cy) * d / camInfo.fy,	d);
	}

	return rs;
}

cv::Vec4f fast_PlaneDetect(const cv::Mat & depthMat, cv::Mat & rgbMat, const ImgProc3D::Intr & camInfo, bool img320x240/* =false */)
{
	cv::RNG rng(0xFFFFFFFF);
	bool planeDetected = false;

	int imgW = depthMat.cols;
	int imgH = depthMat.rows;

	cv::Point2i invalid_loc, p1_loc, p2_loc, p3_loc;
	if (img320x240)	{
		p1_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2, imgH - 30), rng, cv::Point2i(50, 25));
		p2_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2 - 80, imgH / 2), rng, cv::Point2i(50, 20));
		p3_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2 + 80, imgH / 2), rng, cv::Point2i(50, 20));
	}
	else {
		p1_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2, imgH - 60), rng, cv::Point2i(50, 30));
		p2_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2 - 150, imgH / 2), rng, cv::Point2i(50, 20));
		p3_loc = getRandomSample(depthMat, cv::Point2i(imgW / 2 + 150, imgH / 2), rng, cv::Point2i(50, 20));
	}

	if (p1_loc == INVALID_CVPOINT2i || p2_loc == INVALID_CVPOINT2i || p3_loc == INVALID_CVPOINT2i)
	{
		std::cout << "Detect plane failed \n";
		return cv::Vec4f(0.f, 1.f, 0.f, -0.03f); //Get previous values or something else
	}

	//cv::circle(rgbMat, p1_loc, 3, cv::Scalar(0, 0, 255), 3);
	//cv::circle(rgbMat, p2_loc, 3, cv::Scalar(0, 0, 255), 3);
	//cv::circle(rgbMat, p3_loc, 3, cv::Scalar(0, 0, 255), 3);

	cv::Vec3f p1 = getPointXYZ(depthMat, camInfo, p1_loc);
	cv::Vec3f p2 = getPointXYZ(depthMat, camInfo, p2_loc);
	cv::Vec3f p3 = getPointXYZ(depthMat, camInfo, p3_loc);
	cv::Vec3f c = (p1 + p2 + p3) / 3;

	cv::Vec3f v1 = cv::normalize(p2 - p1);
	cv::Vec3f v2 = cv::normalize(p3 - p1);

	cv::Vec3f planeNormal = cv::normalize(v1.cross(v2));
	float d = -planeNormal[0] * c[0] - planeNormal[1] * c[1] - planeNormal[2] * c[2];//d = - ax -by -cz;

	//printf("Model: %f *x + %f *y + %f *z + %f = 0\n", planeNormal[0],planeNormal[1],planeNormal[2],d);
	return cv::Vec4f(planeNormal[0], planeNormal[1], planeNormal[2], d);
}

void convertToXYZ(const cv::Mat & depthMat, const ImgProc3D::Intr & camInfo, cv::Mat & xyzMat)
{
	cv::Mat depthImgF;
	depthMat.convertTo(depthImgF, CV_32F); // convert the image data to float type 
	depthImgF /= camInfo.scale;

	const float qnan = std::numeric_limits<float>::quiet_NaN();

	int idx = 0;
	for (int i = 0; i < depthImgF.rows; i++)
	{
		for (int j = 0; j < depthImgF.cols; j++)
		{
			float d = depthImgF.at<float>(i, j);
			

			if (d == 0)
				xyzMat.at<cv::Point3f>(i, j) = cv::Point3f(qnan, qnan, qnan);
			else
				xyzMat.at<cv::Point3f>(i, j) = cv::Point3f((j - camInfo.cx) * d / camInfo.fx, (i - camInfo.cy) * d / camInfo.fy, d);
			idx++;
		}
	}
}

void fillLaneMap2D(cv::Mat & xyzMat, cv::Mat & lane2D, cv::Vec4f planeModel, float threshold)
{
	cv::Point3f planeNormal(planeModel[0], planeModel[1], planeModel[2]);
	cv::Point3f e_1 = planeNormal.cross(cv::Point3f(0, 0, 1));
	cv::Point3f e_2 = -planeNormal.cross(e_1);
	cv::Point3f planeOrg = -planeModel[3] * planeNormal;

	for (int i = 0; i < xyzMat.rows; i++)
	{
		for (int j = 0; j < xyzMat.cols; j++)
		{
			cv::Point3f p = xyzMat.at<cv::Point3f>(i, j);
			if (!std::isnan(p.x))
			{
				if (fabs(planeModel[0] * p.x + planeModel[1] * p.y + planeModel[2] * p.z + planeModel[3]) < threshold)
				{
					cv::Point3f p_t = p - planeOrg;

					float p_x_new = p_t.dot(e_1);
					float p_y_new = p_t.dot(e_2);
					lane2D.at<cv::Point2f>(i, j) = cv::Point2f(p_x_new, p_y_new);
				}
			}
			/*	NEAR RANGE INTERPOLATION	*/
			/*else{
				if (i > 240){
					const float fx = 570.342165925f;
					const float fy = 570.341946943f;
					const float cx = 319.5f;
					const float cy = 239.5f;
					cv::Point3f rayLine = cv::normalize(cv::Vec3f((j - cx) / fx, (i - cy) / fy, 1));
					float r = rayLine.dot(planeNormal);

					p = (-planeModel[3]) / r * rayLine;
					cv::Point3f p_t = p - planeOrg;
					float p_x_new = p_t.dot(e_1);
					float p_y_new = p_t.dot(e_2);
					lane2D.at<cv::Point2f>(i, j) = cv::Point2f(p_x_new, p_y_new);
				}
			}*/
		}
	}
}

void create2DGrid(cv::Mat & lane2DMap, cv::Mat & colorMap, cv::Mat & gridMap)
{
	// 1 pixel = 5mm
	float scale = 1/0.005;
	int width = gridMap.cols;
	int height = gridMap.rows;
	cv::Rect mapRect(0, 0, width, height);

	for (int i = 0; i < lane2DMap.rows; i++)
	{
		for (int j = 0; j < lane2DMap.cols; j++)
		{
			cv::Point2f p = lane2DMap.at<cv::Point2f>(i, j);
			if (!std::isnan(p.x))
			{
				int new_x = int(width / 2 + p.x * scale);
				int new_y = int(height - p.y * scale);
				if (mapRect.contains(cv::Point2i(new_x, new_y)))
				{
					gridMap.at<cv::Vec3b>(new_y, new_x) = colorMap.at<cv::Vec3b>(i, j);
				}
			}
		}
	}
}
void line_equation(cv::Point p1, cv::Point p2, cv::Point3f& equa) {
    equa.x = p2.y - p1.y;
    equa.y = p1.x - p2.x;
    equa.z = -1 * (equa.x * p1.x + equa.y * p1.y);
}

float cal_gradient(cv::Point3f p) {
    float dy = p.x;
    float dx = -p.y;

    return std::atan2(dy, dx);
}

bool cmp_function(cv::Point3f a, cv::Point3f b) {
    return (a.y > b.y || (a.y == b.y && a.x < b.x));
}

int count_white(const cv::Mat& image, cv::Point topleft, int mask_size) {
    int cnt = 0;
	if (!cv::Rect(0,0,image.cols,image.rows).contains(cv::Point2i(topleft.x + mask_size,topleft.y + mask_size)))
	{
		return 0; //FIX BUGS ACCESS
	}
    for (int i = topleft.y; i < topleft.y + mask_size; i++)
        for (int j = topleft.x; j < topleft.x + mask_size; j++) {
            if (image.at<uchar>(i, j) > 200)
                cnt++;
        }
    return cnt;
}

void line_processing(const cv::Mat& image, cv::Point first, int flag, std::vector<cv::Point>& line, 
    int off_set_x, int off_set_y, int mask_size) 
{
    cv::Point point(first.x, first.y);
    cv::Point best_p(point.x, point.y);
    cv::Point real_p(first.x, first.y);
    int cnt, best_cnt = 0, diff = 0, rows = image.rows, cols = image.cols;
    float thre_lane = mask_size * mask_size * 2 / 10;
    bool real;
    cv::Scalar color;
    line.push_back(first);

    if (flag == 0) {
        color = cv::Scalar(150, 0, 0);
    }
    else {
        color = cv::Scalar(100, 250, 0);
    }

    while (true) {
        point.y -= off_set_y;
        if (point.y < 0)
            break;
        best_cnt = 0;
        real = false;

        int prev_x = point.x;
        for (int k = -5; k <= 5; k++) {
            point.x = prev_x + k * off_set_x;
            if (point.x < 0 || point.x > cols)
                continue;
            cnt = count_white(image, point, 9);
            if (cnt > thre_lane && cnt > best_cnt) {
                best_cnt = cnt;
                best_p.x = point.x;
                best_p.y = point.y;
            }
        }
        
        if (best_cnt > 0) {
            diff = best_p.x - real_p.x;
            point.x = best_p.x;     point.y = best_p.y;
            real_p.x = best_p.x;    real_p.y = best_p.y;
            real = true;
        } else {
            point.x = prev_x + diff;
        }
        
        if (real) {
            // cv::rectangle(input_img, cv::Point(point.x, point.y), 
            //     cv::Point(point.x + mask_size, point.y + mask_size), color, 2);
            line.push_back(point);
        }
        // else
        //     cv::rectangle(input_img, cv::Point(point.x, point.y), 
        //         cv::Point(point.x + mask_size, point.y + mask_size), cv::Scalar(50, 100, 100), 2);
    }
}

void find_initial_point(const cv::Mat& image, cv::Point& best_p_left, cv::Point& best_p_right,
     bool& flag_left, bool& flag_right, int off_set_x, int off_set_y, int mask_size) 
{
    int rows = image.rows, cols = image.cols;
    float thre_white = mask_size*mask_size*1/2;
    cv::Point first(0, image.rows);
    cv::Point point(0, image.rows);
    best_p_left = cv::Point(point.x, point.y);
    best_p_right = cv::Point(point.x, point.y);
    int cnt, best_cnt_left = 0, best_cnt_right = 0;
    
    while (point.y > rows * 2 / 3) {
        point.y -= off_set_y;
        point.x = 0;

        while (point.x < cols) { // ACCESS ERRORS !!!
            point.x = point.x + off_set_x;
			if (point.x < 0 || point.x > cols)
                continue;
            cnt = count_white(image, point, 9);
            if (point.x <= cols / 2) {
                if (flag_left == false && cnt > thre_white && cnt > best_cnt_left) {
                    best_cnt_left = cnt;
                    best_p_left.x = point.x;
                    best_p_left.y = point.y;
                }
            } else {
                if (flag_right == false && cnt > thre_white && cnt > best_cnt_right) {
                    best_cnt_right = cnt;
                    best_p_right.x = point.x;
                    best_p_right.y = point.y;
                }
            }
        }
        
        if (flag_left == false && best_cnt_left > 0) {
            flag_left = true;
            // cv::rectangle(input_img, cv::Point(best_p_left.x, best_p_left.y), 
            //     cv::Point(best_p_left.x + mask_size, best_p_left.y + mask_size), cv::Scalar(150, 0, 0), 7);
        }

        if (flag_right == false && best_cnt_right > 0) {
            flag_right = true;
            // cv::rectangle(input_img, cv::Point(best_p_right.x, best_p_right.y), 
            //     cv::Point(best_p_right.x + mask_size, best_p_right.y + mask_size), cv::Scalar(100, 250, 0), 7);
        }
    }
}

void find_lane(const cv::Mat& image, std::vector<cv::Point>& left_line, std::vector<cv::Point>& right_line) {
    cv::Point best_p_left(0, 0);
    cv::Point best_p_right(0, 0);
    bool flag_left = false, flag_right = false;

    find_initial_point(image, best_p_left, best_p_right, flag_left, flag_right, 3, 11);
    
    if (flag_left)   
        line_processing(image, best_p_left, 0, left_line, 3, 11);
    if (flag_right)   
        line_processing(image, best_p_right, 1, right_line, 3, 11);
}

std::vector<cv::Point3f> find_lane_center(const cv::Mat& image, int radius, int mask_size) {
    float a, b;
    cv::Point center(0, 0);
    std::vector<cv::Point> left_line, right_line;
    std::vector<cv::Point3f> centers, centers_cmb;
    cv::Scalar color;
    cv::Point prev_p;
    cv::Point3f line_eq;

    find_lane(image, left_line, right_line);

    if (left_line.size() > right_line.size()) {
    // center from left
        color = cv::Scalar(150, 0, 0);
        for (int i = 1; i < left_line.size(); i++) 
        {
            line_equation(left_line[i - 1], left_line[i], line_eq);
            float tmp = sqrt(line_eq.x * line_eq.x + line_eq.y * line_eq.y);
            center.y = left_line[i - 1].y - line_eq.y * radius / tmp;
            center.x = left_line[i - 1].x + mask_size - line_eq.x * radius / tmp;
            centers.push_back(cv::Point3f(center.x, center.y, 0));
        }
    } else {
    // center from right
        color = cv::Scalar(100, 250, 0);
        for (int i = 1; i < right_line.size(); i++) 
        {
            line_equation(right_line[i - 1], right_line[i], line_eq);
            float tmp = sqrt(line_eq.x * line_eq.x + line_eq.y * line_eq.y);
            center.y = right_line[i - 1].y + line_eq.y * radius / tmp;
            center.x = right_line[i - 1].x + line_eq.x * radius / tmp;
            centers.push_back(cv::Point3f(center.x, center.y, 1));
        }
    }        

    // combine center
    std::sort(centers.begin(), centers.end(), cmp_function);

    // fix bug
    if (centers.size() == 0)
        return centers;

    prev_p = cv::Point(centers[0].x, 1000);
    for (int i = 0; i < centers.size(); i++)  
        if (prev_p.y - centers[i].y >= 15)
    {
        prev_p.x = centers[i].x;
        prev_p.y = centers[i].y;
        centers_cmb.push_back(cv::Point3f(centers[i].x, 512 - centers[i].y, centers[i].z));
    }

    return centers_cmb;
}

cv::Point3f fit_lane(std::vector<cv::Point3f> centers, int range, int left, bool& flag) {
    float thre_dis = 2.0f, thre_num = range * 6 / 10;
    //int cnt = 0;
    cv::Point3f equ;

    for (int i = left; i < left + range; i++)
        for (int j = i + 1; j < left + range; j++) {
            line_equation(cv::Point(centers[i].x, centers[i].y), 
                            cv::Point(centers[j].x, centers[j].y), equ);
            int cnt = 0;

            for (int k = left; k < left + range; k++) {
                float dis = std::fabs(centers[k].x*equ.x + centers[k].y*equ.y + equ.z) 
                                        / std::sqrt(equ.x*equ.x + equ.y*equ.y);
                if (dis < thre_dis)
                    cnt++;
            }

            if (cnt > thre_num) {
                flag = true;
                return equ;
            }
        }
}

bool intersect(cv::Point3f p1, cv::Point3f p2, cv::Point2f& intersP) {
    float epslon = 0.05f;
    if ((-p1.x * p2.z - p2.x * p1.z) <= epslon)
        return false;
    intersP.y = (p1.x * p2.z - p2.x * p1.z) / (p2.x * p1.y - p1.x * p2.y);
    intersP.x = (-p1.z - p1.y * intersP.y) / p1.x;
    return true;
}

bool find_curve_point(const cv::Mat& image, cv::Point2f& intersP) {
    std::vector<cv::Point3f> centers = find_lane_center(image, 65); 

    int breakP = -1;
    for (int i = 0; i <centers.size(); i++) 
        if (std::abs(centers[i].y - centers[0].y) > 150) {
             breakP = i - 1;
             break;
        }
    if (breakP == -1)
        return false;

    cv::Point3f line1, line2;
    cv::Point2f p1, p2;

    bool flag1 = false, flag2 = false;

    std::cout << "Line1: " << std::endl;
    line1 = fit_lane(centers, breakP, 0, flag1);

    std::cout << "Line2: " << std::endl;
    line2 = fit_lane(centers, centers.size() - breakP, breakP, flag2);

    float gra1, gra2;

    if (flag1 && flag2) {
        p1.x = 0;   p1.y = -line1.z / line1.y;
        p2.x = 512, p2.y = -(line1.z + line1.x * p2.x) / line1.y;

        p1.x = 0;   p1.y = -line2.z / line2.y;
        p2.x = 512, p2.y = -(line2.z + line2.x * p2.x) / line2.y;

        gra1 = cal_gradient(line1);
        gra2 = cal_gradient(line2);

        if (intersect(line1, line2, intersP)) {
            if (fabs(gra1 - gra2) > 0.3f) {
                return true;
            }
        }
    }

    return false;
}
