#ifndef __LANE_DETECTOR_H
#define __LANE_DETECTOR_H

#include "ImgProc.h"

namespace ImgProc3D
{
	enum IMAGE_SOURCE
	{
		SOURCE_RECORDED_IMG = 0,
		SOURCE_RAW_IMG = 1
	};

	enum RACECAR_MODE
	{
		RACECAR_MODE_NORMAL = 0,
		RACECAR_MODE_AVOID_OBJECT = 1
	};


	class LaneDetector
	{
	public:
		LaneDetector(int imgWidth, int imgHeight);
		~LaneDetector();

		void setCamera(ImgProc3D::IntrMode _mode);
		void processFrame(cv::Mat & rgbMat, cv::Mat & dMat); // DO SOMETHING

		cv::Mat xyzMap,laneMap2D;
		bool smallImage;
	private:
		ImgProc3D::Intr m_camInfo;
		IMAGE_SOURCE m_img_source;
		cv::Vec4f m_previous_plane_model;

		cv::Mat laneProject2D;
		void fillLaneMap();
		void detectLaneCenter();
	};
}


#endif