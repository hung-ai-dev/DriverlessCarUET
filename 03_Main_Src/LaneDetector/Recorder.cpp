#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "ImgProc/LaneDetector.h"
#include <stdlib.h>
#include <ctime>

#include "OpenNI2_Helper.h"

bool programShouldClose;

std::string dirPath = "Sample/";
std::string depthDir = "depth/";
std::string colorDir = "rgb/";
std::ofstream recordFile;

openni::Device	* p_device;

#define RAW_WIDTH 320
#define RAW_HEIGHT 240
#define FPS 30
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000 ms

void setupInputFolder();

int main()
{
	ImgProc3D::LaneDetector lane_detector(RAW_WIDTH, RAW_HEIGHT);
	lane_detector.setCamera(ImgProc3D::IntrMode_320x240_RAW);
	openni::VideoStream	depthStream, colorStream;
	openni::VideoFrameRef		m_depthFrame, m_colorFrame;

	openni::VideoStream * streams[] = { &depthStream, &colorStream };
	setupAstra(p_device, depthStream, colorStream, RAW_WIDTH, RAW_HEIGHT, FPS);

	bool programShouldClose = false;
	bool recording = false;
	int indexData = 0;

	while (!programShouldClose)
	{
		int changedStreamDummy;
		char rc = openni::OpenNI::waitForAnyStream(streams, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
		if (rc != openni::STATUS_OK){
			printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, openni::OpenNI::getExtendedError());
			continue;
		}
		if (rc != openni::STATUS_OK){
			printf("Read failed!\n%s\n", openni::OpenNI::getExtendedError());
			continue;
		}

		int64 st = cv::getTickCount();
		colorStream.readFrame(&m_colorFrame);
		depthStream.readFrame(&m_depthFrame);

		cv::Mat depthMat = cv::Mat(RAW_HEIGHT, RAW_WIDTH, CV_16UC1, (openni::DepthPixel *)m_depthFrame.getData());
		//depthMat *= 5;

		cv::Mat bgrMat, gray_img;
		cv::Mat raw_colorMat = cv::Mat(RAW_HEIGHT, RAW_WIDTH, CV_8UC3, (uint8_t *)m_colorFrame.getData());
		cv::cvtColor(raw_colorMat, bgrMat, CV_RGB2BGR);
		lane_detector.processFrame(bgrMat, depthMat);

		cv::cvtColor(lane_detector.laneMap2D, gray_img, cv::COLOR_BGR2GRAY);
		lane_detector.calibFrame(bgrMat, depthMat);

		std::vector<cv::Point2f> centers = lane_detector.findLaneCenter(gray_img, 65);
		
		for (int i = 0; i < centers.size(); i++)
		{
			cv::circle(lane_detector.laneMap2D, cv::Point2f(centers[i].x, 512 - centers[i].y), 3, cv::Scalar(100, 250, 0), 2);
		}
		
		
		if (recording)
		{
			std::string depFile = depthDir + std::to_string(indexData) + ".png";
			std::string rgbFile = colorDir + std::to_string(indexData) + ".png";
			recordFile << std::to_string(indexData) << " " << depFile << " " << std::to_string(indexData) << " " << rgbFile << "\n";

			cv::imwrite(dirPath + depFile, depthMat);
			cv::imwrite(dirPath + rgbFile, bgrMat);
			indexData++;
		}

		cv::imshow("Depth", depthMat);
		cv::imshow("RGB", raw_colorMat);
		cv::imshow("lane", lane_detector.laneMap2D);
		cv::imshow("t", lane_detector.objectMask);

		char key = cv::waitKey(5);
		if (key == 'q') programShouldClose = true;
		if (key == 'r') {
			setupInputFolder();
			recording = true;
		}
	}

	depthStream.stop();
	depthStream.destroy();
	colorStream.stop();
	colorStream.destroy();
	
	openni::OpenNI::shutdown();
	recordFile.close();
	return 0;
}

void setupInputFolder()
{
	system("mkdir Sample && cd ./Sample && mkdir depth && mkdir rgb && cd ..");
	recordFile.open(dirPath + "associations.txt");

	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	recordFile << "# RGBD recorder TUM-format (Scale = x5 millimeter = 5000 ) \n";
	// 	recordFile << "# Date: " << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << std::endl;
}