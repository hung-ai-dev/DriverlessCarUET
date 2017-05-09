#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
#include <stdlib.h>

bool programShouldClose;

std::string dirPath = "Sample/";
std::string depthDir = "depth/";
std::string colorDir = "rgb/";
std::ofstream recordFile;

void setupInputFolder();
int runRealsense();
int runOpenNI2();

int main()
{
	//runRealsense();
	runOpenNI2();
}

//#include "librealsense/rs.hpp"
//int runRealsense()
//{
//	cv::Mat depthMat, colorMat;
//	
//	setupInputFolder();
//
//	// Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
//	rs::log_to_console(rs::log_severity::warn);
//	// Create a context object. This object owns the handles to all connected realsense devices.
//	rs::context ctx;
//
//	printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
//	if (ctx.get_device_count() == 0) return EXIT_FAILURE;
//
//	// This tutorial will access only a single device, but it is trivial to extend to multiple devices
//	rs::device * dev = ctx.get_device(0);
//	printf("\nUsing device 0, an %s\n", dev->get_name());
//	printf("    Serial number: %s\n", dev->get_serial());
//	printf("    Firmware version: %s\n", dev->get_firmware_version());
//	printf("	Press R to start Recording ... \n");
//
//	// Configure depth and color to run with the device's preferred settings
//	dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
//	dev->enable_stream(rs::stream::color, rs::preset::best_quality);
//	dev->start();
//
//	// Retrieve camera parameters for mapping between depth and color
//	rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
//	rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
//	rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
//	
//	recordFile << "# Depth instr: " << std::fixed
//		<< color_intrin.width << " "
//		<< color_intrin.height << " "
//		<< color_intrin.fx << " "
//		<< color_intrin.fy << " "
//		<< color_intrin.ppx << " "
//		<< color_intrin.ppy << "\n";
//
//	int indexData = 0;
//	bool isRecording = false;
//	while (!programShouldClose)
//	{
//		dev->wait_for_frames();
//
//		// Retrieve our images
//		//const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
//		//const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);
//		depthMat = cv::Mat(color_intrin.height, color_intrin.width, CV_16UC1, (ushort*)dev->get_frame_data(rs::stream::depth_aligned_to_color));
//		colorMat = cv::Mat(color_intrin.height, color_intrin.width, CV_8UC3, (uchar*)dev->get_frame_data(rs::stream::color));
//
//		depthMat *= 5;
//		cv::Mat bgrMat;
//		cv::cvtColor(colorMat, bgrMat, cv::COLOR_RGB2BGR);
//
//		cv::imshow("depth IMG", depthMat);
//		cv::imshow("color IMG", bgrMat);
//
//		float scale = dev->get_depth_scale();
//
//		
//		/* === Record: === */
//		if (isRecording)
//		{
//			std::string depFile = depthDir + std::to_string(indexData) + ".png";
//			std::string rgbFile = colorDir + std::to_string(indexData) + ".png";
//			recordFile << std::to_string(indexData) << " " << depFile << " " << std::to_string(indexData) << " " << rgbFile << "\n";
//
//			cv::imwrite(dirPath + depFile, depthMat);
//			cv::imwrite(dirPath + rgbFile, bgrMat);
//			indexData++;
//		}
//		
//		char key = cv::waitKey(5);
//		if (key == 'q'){ programShouldClose = true; }
//		if (key == 'r'){ isRecording = true; } // Press R to start Recording
//	}
//	recordFile.close();
//	return 0;
//}

#include "OpenNI.h"
#include "NISensorController.h"

int runOpenNI2()
{
	setupInputFolder();
	NISensorController ni_sensor;
	ni_sensor.initKinect();
	
	recordFile << "# Img instr: " << std::fixed
		<< ni_sensor.colorStream.getVideoMode().getResolutionX() << " "
		<< ni_sensor.colorStream.getVideoMode().getResolutionY() << " "
		<< ni_sensor.colorStream.getHorizontalFieldOfView() << " "
		<< ni_sensor.colorStream.getVerticalFieldOfView() << " "
		<< ni_sensor.colorStream.getVideoMode().getResolutionX() / 2 << " "
		<< ni_sensor.colorStream.getVideoMode().getResolutionY() / 2 << std::endl;

	cv::Mat bgrMat = cv::Mat(RAW_HEIGHT, RAW_WIDTH, CV_8UC3);
	bool shoudClose = false;
	bool isRecording = false;
	int indexData = 0;

	while (!shoudClose)
	{
		if (ni_sensor.hasNewframe)
		{
			cv::Mat depthMat = cv::Mat(RAW_HEIGHT, RAW_WIDTH, CV_16UC1, ni_sensor.pDepth);
			depthMat *= 5;
			cv::Mat colorRaw = cv::Mat(RAW_HEIGHT, RAW_WIDTH, CV_8UC3, ni_sensor.pColor);
			cvtColor(colorRaw, bgrMat, CV_RGB2BGR);
			cv::imshow("cl", bgrMat);
			cv::imshow("d", depthMat);
			
			if (isRecording)
			{
				std::string depFile = depthDir + std::to_string(indexData) + ".png";
				std::string rgbFile = colorDir + std::to_string(indexData) + ".png";
				recordFile << std::to_string(indexData) << " " << depFile << " " << std::to_string(indexData) << " " << rgbFile << "\n";

				cv::imwrite(dirPath + depFile, depthMat);
				cv::imwrite(dirPath + rgbFile, bgrMat);
				indexData++;
			}
			ni_sensor.hasNewframe = false;
		}
		uchar eKey = cv::waitKey(5);
		if (eKey == 'q'){ shoudClose = true; }
		if (eKey == 'r'){ isRecording = true; } // Press R to start Recording
	}

	ni_sensor.closeKinect();
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
