#include <cstdio>
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include "opencv2/opencv.hpp"

// hard coded
// const int mask_size = 9, off_set_y = 11, off_set_x = 3, radius = 65;

cv::Mat input_img, gray_img;

int count_white(const cv::Mat& image, cv::Point topleft, int mask_size = 9) {
    int cnt = 0;
	if (!cv::Rect(0,0,image.cols,image.rows).contains(cv::Point2i(topleft.x + mask_size,topleft.y + mask_size)))
	{
		return 0; //FIX BUGS ACCESS
	}
    for (int i = topleft.y; i < topleft.y + mask_size; i++)
        for (int j = topleft.x; j < topleft.x + mask_size; j++) {
            if (image.at<uchar>(i, j) > 150)
                cnt++;
        }
    return cnt;
}

void line_equation(cv::Point p1, cv::Point p2, cv::Point3f& equa) {
    equa.x = p2.y - p1.y;
    equa.y = p1.x - p2.x;
    equa.z = -1 * (equa.x * p1.x + equa.y * p1.y);
}

bool cmp_function(cv::Point3f a, cv::Point3f b) {
    return (a.y > b.y || (a.y == b.y && a.x < b.x));
}

float cal_gradient(cv::Point3f p) {
    float dy = p.x;
    float dx = -p.y;

    return std::atan2(dy, dx);
}

void line_processing(const cv::Mat& image, cv::Point first, int flag, std::vector<cv::Point>& line, 
    int off_set_x, int off_set_y, int mask_size = 9) 
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
            cv::rectangle(input_img, cv::Point(point.x, point.y), 
                cv::Point(point.x + mask_size, point.y + mask_size), color, 2);
            line.push_back(point);
        }
        else
            cv::rectangle(input_img, cv::Point(point.x, point.y), 
                cv::Point(point.x + mask_size, point.y + mask_size), cv::Scalar(50, 100, 100), 2);
    }
}

void find_initial_point(const cv::Mat& image, cv::Point& best_p_left, cv::Point& best_p_right,
     bool& flag_left, bool& flag_right, int off_set_x, int off_set_y, int mask_size = 9) 
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
            cv::rectangle(input_img, cv::Point(best_p_left.x, best_p_left.y), 
                cv::Point(best_p_left.x + mask_size, best_p_left.y + mask_size), cv::Scalar(150, 0, 0), 7);
        }

        if (flag_right == false && best_cnt_right > 0) {
            flag_right = true;
            cv::rectangle(input_img, cv::Point(best_p_right.x, best_p_right.y), 
                cv::Point(best_p_right.x + mask_size, best_p_right.y + mask_size), cv::Scalar(100, 250, 0), 7);
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

std::vector<cv::Point3f> find_lane_center(const cv::Mat& image, int radius, int mask_size = 9) {
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
        centers_cmb.push_back(cv::Point3f(centers[i].x, centers[i].y, centers[i].z));
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

void find_curve_point(const cv::Mat& image) {
    std::vector<cv::Point3f> centers = find_lane_center(image, 65); 

    for (int i = 0; i < centers.size(); i++) {
        cv::circle(input_img, cv::Point(centers[i].x, centers[i].y), 3, cv::Scalar(0, 0, 255), 2);
    }

    int breakP = -1;
    for (int i = 0; i <centers.size(); i++) 
        if (std::abs(centers[i].y - centers[0].y) > 150) {
             breakP = i - 1;
             break;
        }
    if (breakP == -1)
        return;

    cv::Point3f line1, line2;
    cv::Point2f p1, p2;

    bool flag1 = false, flag2 = false;

    std::cout << "Line1: " << std::endl;
    line1 = fit_lane(centers, breakP, 0, flag1);

    std::cout << "Line2: " << std::endl;
    line2 = fit_lane(centers, centers.size() - breakP, breakP, flag2);

    cv::Point2f intersP;
    float gra1, gra2;

    if (flag1 && flag2) {
        p1.x = 0;   p1.y = -line1.z / line1.y;
        p2.x = 512, p2.y = -(line1.z + line1.x * p2.x) / line1.y;

        cv::line(input_img, p1, p2, cv::Scalar(255, 0, 0), 2);

        p1.x = 0;   p1.y = -line2.z / line2.y;
        p2.x = 512, p2.y = -(line2.z + line2.x * p2.x) / line2.y;

        cv::line(input_img, p1, p2, cv::Scalar(0, 0, 255), 2);

        gra1 = cal_gradient(line1);
        gra2 = cal_gradient(line2);
        std::cout << gra1 << " " << gra2 << " " << gra1 - gra2 << std::endl;

        if (intersect(line1, line2, intersP)) {
            if (fabs(gra1 - gra2) > 0.3f) {
                cv::circle(input_img, intersP, 3, cv::Scalar(100, 150, 150), 2);
            }
        }
    }
}

int main(int argc, char* argv[]) {
	std::string path = "/home/hungnd/Desktop/MagicalRaceUET----/DataCDS/Sample-02/lane/";
	std::string file_path;

    char key = 0;
    
    for (int fr = 0; fr < 1000 ; fr++) {
        file_path = path + std::to_string(fr) + ".png";
        std::cout << file_path << std::endl;
        input_img = cv::imread(file_path);

        // cv::imshow("input", input_img);
        // video >> input_img;

        cv::cvtColor(input_img, gray_img, cv::COLOR_BGR2GRAY);
        find_curve_point(gray_img);

        cv::imshow("in", input_img);
        key = cv::waitKey(-1);
        if( key == 27 ) break;
    }

    return 0;
}