#include "Utilities/DatasetReader.h"
#include <Eigen/Dense>
#include "ImgProc/LaneDetector.h"
#include "opencv2/opencv.hpp"
#include "queue"

#define RAW_WIDTH 320
#define RAW_HEIGHT 240	

std::vector<FullFrame> frames;
std::string dirPath = "/home/ubuntu/Desktop/MagicalRaceUET/03_Main_Src/LaneDetector/build/Sample99/";
int idx = 0;

double getThetaFromObj(cv::Point2f dst) {
	double ans = atan2(abs(dst.x), dst.y) * 180 / M_PI;
	return ans;
}
int main()
{
	readSyncFileHeader(dirPath + "associations.txt", frames);
	ImgProc3D::LaneDetector lane_detector(RAW_WIDTH, RAW_HEIGHT);
	lane_detector.setCamera(ImgProc3D::IntrMode_320x240_RAW);

	int cur_radius = 65;
	bool useLeftLane = 0;
	bool useRightLane = 0;

	float et, st;

	while (idx < frames.size())
	{
		idx++;
		printf("frame no %d / %d \n", idx, frames.size());

		cv::Mat bgrMat = cv::imread(dirPath + frames[idx].rgbHeaders.filePath);
		cv::Mat dMat = cv::imread(dirPath + frames[idx].depthHeader.filePath, CV_LOAD_IMAGE_ANYDEPTH);

		et = cv::getTickCount();

		lane_detector.processFrame(bgrMat, dMat);

		cv::imshow("ori_lane", lane_detector.laneMap2D);

		cv::Mat gray_img;
		cv::cvtColor(lane_detector.laneMap2D, gray_img, cv::COLOR_BGR2GRAY);
		std::cout << cur_radius << " " << useLeftLane << " " << useRightLane << '\n';

		lane_detector.findLane(gray_img);
		std::vector<cv::Point2f> centers = lane_detector.findLaneCenter(gray_img, 50, 0, 0);

/*
		if (lane_detector.iniLeft.x != -1) {
			for (int i = 0; i < lane_detector.iniLefts.size(); i++) {
				cv::rectangle(lane_detector.laneMap2D, lane_detector.iniLefts[i], 
					cv::Point(lane_detector.iniLefts[i].x + 11, lane_detector.iniLefts[i].y + 11),
					cv::Scalar(150, 0, 0), 7);
			}

			for (int i = 0; i < lane_detector.left_line.size(); i++)
				cv::rectangle(lane_detector.laneMap2D, lane_detector.left_line[i], 
						cv::Point(lane_detector.left_line[i].x + 11, lane_detector.left_line[i].y + 11),
						cv::Scalar(150, 0, 0), 2);
		}

		if (lane_detector.iniRight.x != -1) {
			for (int i = 0; i < lane_detector.iniRts.size(); i++) {
				cv::rectangle(lane_detector.laneMap2D, lane_detector.iniRts[i], 
					   cv::Point(lane_detector.iniRts[i].x + 11, lane_detector.iniRts[i].y + 11), 
			   		cv::Scalar(100, 250, 0), 7);
			}
			   
			for (int i = 0; i < lane_detector.right_line.size(); i++)
				cv::rectangle(lane_detector.laneMap2D, lane_detector.right_line[i], 
						cv::Point(lane_detector.right_line[i].x + 11, lane_detector.right_line[i].y + 11),
						cv::Scalar(250, 100, 0), 2);
		}
*/
		if (centers.size() > 1) {
			for (int i = 0; i < centers.size(); i++)
			{
				cv::circle(lane_detector.laneMap2D, cv::Point2f(centers[i].x, 512 - centers[i].y), 3, cv::Scalar(100, 250, 0), 2);
			}

//			cv::imshow("lane1", lane_detector.laneMap2D);
			//DO SOMETHING:
			cv::Point objLeft;
			cv::Point objRight;
			cv::Point xy = lane_detector.projectToMap2D(dMat, objLeft, objRight);
			std::cout << xy.x << " " << xy.y << '\n';
			if (xy.x >= 0 && (512.0 - xy.y) * 0.5 > 30.0) {
				cv::circle(lane_detector.laneMap2D, xy, 3, cv::Scalar(0, 0, 255), 2);
				cv::circle(lane_detector.laneMap2D, objLeft, 3, cv::Scalar(100, 100, 0), 2);
				cv::circle(lane_detector.laneMap2D, objRight, 3, cv::Scalar(100, 100, 0), 2);

				int objLeftRight = lane_detector.checkObsPosition(lane_detector.left_line, lane_detector.right_line, xy);
				if (checkObjOutOfRoad(objLeft, objRight, objLeftRight, centers) == 0) {
					cv::Point2f obs = cv::Point2f((xy.x - 256) * 0.5, (512.0 - xy.y) * 0.5);
					
					std::cout << "-1: Left / 1: Right => " << objLeftRight << std::endl;
					std::cout << "Obstacle: " << obs << std::endl;
					
					cur_radius = 30;	
					if (objLeftRight == -1) {
						useLeftLane = 0;
						useRightLane = 1;
					} else {
						useLeftLane = 1;
						useRightLane = 0;
					}
					std:: cout << "Infor : " << useLeftLane << " " << useRightLane << '\n';
					std::vector<cv::Point2f> centers1 = lane_detector.findLaneCenter(gray_img, cur_radius, useLeftLane, useRightLane);
					for (int i = 0; i < centers1.size(); i++)
					{
						cv::circle(lane_detector.laneMap2D, cv::Point2f(centers1[i].x, 512 - centers1[i].y), 3, cv::Scalar(0, 250, 255), 2);
					}
				} else {
					std :: cout << "Obj out of road" << '\n';
					cur_radius = 65;
					useLeftLane = 0;
					useRightLane = 0;
				}
			} else {
				std::cout << "No Object" << '\n';
				cur_radius = 65;
				useLeftLane = 0;
				useRightLane = 0;
			}
	
		} else
			std :: cout << "No center" << '\n';
		st = cv::getTickCount();

		std::cout << "Time: " << (st - et) / cv::getTickFrequency() << std::endl;

		cv::imshow("lane", lane_detector.laneMap2D);
		cv::imshow("bgr", bgrMat);
		cv::imshow("t", lane_detector.objectMask);		
		cv::waitKey();
	}
	return 0;
}
