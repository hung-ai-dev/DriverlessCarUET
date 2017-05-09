#include "ImgProc.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <queue>
#include <opencv2/opencv.hpp>

// #define DEBUG
#ifdef DEBUG
    #define debug(x) cerr << #x << ": " << x << std::endl;
#else
    #define debug(x)
#endif

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
cv::Point getCenterObj(cv::Mat & ObjMask, cv::Point & left, cv::Point &right) {
	left = cv::Point(-1, -1);
	right = cv::Point(-1, -1);
	std::vector< std::vector< bool > > state;   state.clear();
	std::vector<int> cntPoint;                  cntPoint.clear();
	std::vector<int> posX;                      posX.clear();
	std::vector<int> posY;                      posY.clear();
	std::vector<cv::Point> objLeft;				objLeft.clear();
	std::vector<cv::Point> objRight;			objRight.clear();
	int row = ObjMask.rows;
	int col = ObjMask.cols;
	int dx[] = {-1, 1, 0, 0};
	int dy[] = {0, 0, -1, 1};

	state = std::vector< std::vector<bool> > (row, std::vector<bool>(col, false));
	
	for (int y = 0; y < row; ++y)
	for (int x = 0; x < col; ++x)
		if (ObjMask.at<uchar>(y, x) == 255 && state[y][x] == false) {
			state[y][x] = true;
			std::queue< std::pair<int, int > > curQ;
			int curSumX = 0;
			int curSumY = 0;
			int curCntP = 0;
			int curX, curY;
			int minX = 1e9, maxX = -1e9, minY, maxY;
			curQ.push( std::make_pair(y , x) );
			while (!curQ.empty()) {
				curY = curQ.front().first;
				curX = curQ.front().second;
				curQ.pop();

				int neighs = 0;
				for(int dir = 0; dir < 4; ++dir) {
					int ny = curY + dy[dir];
					int nx = curX + dx[dir];
					if (0 <= ny && ny < row && 0 <= nx && nx < col) {
						neighs += (ObjMask.at<uchar>(ny, nx) > 0);
					}
				}	
				if (neighs < 3)	continue;

				curSumX += curX;
				curSumY += curY;
				curCntP += 1;
				if (minX > curX) {
					minX = curX;
					minY = curY;
				}
				if (maxX < curX) {
					maxX = curX;
					maxY = curY;
				}
				for(int dir = 0; dir < 4; ++dir) {
					int ny = curY + dy[dir];
					int nx = curX + dx[dir];
					if (0 <= ny && ny < row && 0 <= nx && nx < col) {
						if (state[ny][nx] == false && ObjMask.at<uchar>(ny, nx) > 0) {
							curQ.push(std::make_pair(ny, nx));
							state[ny][nx] = true;
						}
					}
				}

			}
			if (curCntP == 0) continue;
			cntPoint.push_back(curCntP);
			posX.push_back(curSumX / curCntP);
			posY.push_back(curSumY / curCntP);
			objLeft.push_back(cv::Point(minX, minY));
			objRight.push_back(cv::Point(maxX, maxY));
		}
	if (cntPoint.empty()) return cv::Point(-1, -1);
	int len = cntPoint.size(), pmax = 0;
	for (int i = 1; i < len; i++) {
		if (cntPoint[i] > cntPoint[pmax]) pmax = i;
	}
	if (cntPoint[pmax] < 100) return cv::Point(-1, -1);
	int retX = posX[pmax];  int retY = posY[pmax];
	left = objLeft[pmax];
	right = objRight[pmax];
	if (ObjMask.at<uchar>(retY, retX) == 0) {
		int dis = 1; 
		while (ObjMask.at<uchar>(retY, retX) == 0) {
			for (int yy = -dis; yy < dis; yy++) {
				int xx = dis - abs(yy);
				if (retY + yy >= 0 && retY + yy < row && retX + xx >= 0 & retX + xx < col) {
					if (ObjMask.at<uchar>(retY + yy, retX + xx) == 255) {
						retY += yy;
						retX += xx;
						break;
					}
				}
				if (retY - yy >= 0 && retY - yy < row && retX - xx >= 0 & retX - xx < col) {
					if (ObjMask.at<uchar>(retY - yy, retX - xx) == 255) {
						retY -= yy;
						retX -= xx;
						break;
					}
				}
			}
			dis++;
		}
	}
	return cv::Point(retX, retY);
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
		// std::cout << "Detect plane failed \n";
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
cv::Point3f projectPointToPlane(cv::Point3f P, cv::Vec4f plane) {
    double t = (- plane[3] - plane[0]*P.x - plane[1]*P.y - plane[2]*P.z) / 
               (plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]) ;
    cv::Point3f ans;
    ans.x = P.x + plane[0] * t;
    ans.y = P.y + plane[1] * t;
    ans.z = P.z + plane[2] * t;
    return ans;
}
cv:: Point findPointIn2dMap(cv::Point3f p, cv::Vec4f planeModel, int width, int height ) {
    cv::Point3f planeNormal(planeModel[0], planeModel[1], planeModel[2]);
	cv::Point3f e_1 = planeNormal.cross(cv::Point3f(0, 0, 1));
	cv::Point3f e_2 = -planeNormal.cross(e_1);
	cv::Point3f planeOrg = -planeModel[3] * planeNormal;

    cv::Point3f p_t = p - planeOrg;

	float p_x_new = p_t.dot(e_1);
	float p_y_new = p_t.dot(e_2);

    float scale = 1/0.005;

    int new_x = int(width / 2 + p_x_new * scale);
	int new_y = int(height - p_y_new * scale);
    return cv::Point(new_x, new_y);
}
double projectorDistance(std::vector<cv::Point2f> centers, cv::Point2f p) {
    // No center -> no lane -> may be obstacle blind car's vision
    if (centers.empty())
        return 0;
    if (centers.size() == 1) {
        double x = centers[0].x, y = centers[0].y;
        return sqrt((p.x - x) * (p.x - x) + (p.y - y) * (p.y - y));
    }
	
    /*cv::Vec4f line;    
	std::cout << line[0] << ' ' << line[1] << ' ' << line[2] << ' ' << line[3] << std::endl;
    double dArea = fabs(line[1] * (p.x - line[3]) - line[0] * (p.y - line[2]));
	std::cout << "D: " << dArea << std::endl;
    return dArea;*/
	double p1x = centers[0].x, p1y = centers[0].y;
	double p2x = centers.back().x, p2y = centers.back().y;
	double dArea = fabs((p2x - p1x) * (p.y - p1y) - (p2y - p1y) * (p.x - p1x));
	return dArea / sqrt((p1x - p2x) * (p1x - p2x) + (p1y - p2y) * (p1y - p2y));
}

bool checkObjOutOfRoad(cv::Point objLeft, cv::Point objRight, int objLeftRight, std::vector<cv::Point2f> centers) {
	int len = centers.size();
	double u, v;
	double minDist = 1e9;
	objLeft.x = (objLeft.x - 256.0) * 0.5;
	objLeft.y = (512.0 - objLeft.y) * 0.5;
	objRight.x = (objRight.x - 256.0) * 0.5;
	objRight.y = (512.0 - objRight.y) * 0.5;

	double objSize = std::sqrt(sqr(objLeft.x - objRight.x) + sqr(objLeft.y - objRight.y));
	double threshSize = 40;

	///if (objSize > threshSize)
		//return 1;

	for (int i = 0; i < len; i++) {
		centers[i].x = (centers[i].x - 256.0) * 0.5;
		centers[i].y = centers[i].y * 0.5;
	}
	if (objLeftRight == -1) {
		minDist = projectorDistance(centers, objRight);
	} else { 
		minDist = projectorDistance(centers, objLeft);
	}
 	//std :: cout << "distance : " << minDist << '\n';
	//std :: cerr << "Dist from Obj to Center : " << minDist << '\n';
	//std :: cerr << "Position center " << u << " " << v << '\n';
	if (minDist > 30) return 1;
	return 0;
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

float sqr(float a) {
	return a*a;
}