#ifndef __IMG_PROC_H
#define __IMG_PROC_H

#include <iostream>
#include <vector>
#include <opencv2/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <Eigen/Dense>
#include "types.h"

cv::Vec4f fast_PlaneDetect(const cv::Mat & depthMat, cv::Mat & rgbMat, const ImgProc3D::Intr & camInfo, bool img320x240=false);

//CPU CODE
void convertToXYZ(const cv::Mat & depthMat, const ImgProc3D::Intr & camInfo, cv::Mat & xyzMat);
void fillLaneMap2D(cv::Mat & xyzMat, cv::Mat & lane2D, cv::Vec4f planeModel, float threshold=0.1f);
void create2DGrid(cv::Mat & lane2DMap, cv::Mat & colorMap, cv::Mat & gridMap);

cv::Point3f getPointXYZ(const cv::Mat & depthMat, const ImgProc3D::Intr & camInfo, cv::Point2i p);
cv::Point3f projectPointToPlane(cv::Point3f P, cv::Vec4f plane);
cv:: Point findPointIn2dMap(cv::Point3f p, cv::Vec4f planeModel, int width, int height );
cv::Point getCenterObj(cv::Mat & ObjMask, cv::Point & left, cv::Point & right);
bool checkObjOutOfRoad(cv::Point objLeft, cv::Point objRight, int objLeftRight, std::vector<cv::Point2f> centers);
double projectorDistance(std::vector<cv::Point2f> centers, cv::Point2f p);
//HUNG CODE:
void line_equation(cv::Point p1, cv::Point p2, cv::Point3f& equa);
float cal_gradient(cv::Point3f p);
bool cmp_function(cv::Point3f a, cv::Point3f b);
int count_white(const cv::Mat& image, cv::Point topleft, int mask_size = 9);
float sqr(float a);

#endif