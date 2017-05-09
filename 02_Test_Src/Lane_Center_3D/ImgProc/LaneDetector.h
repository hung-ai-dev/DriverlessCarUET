#ifndef __LANE_DETECTOR_H
#define __LANE_DETECTOR_H

#include "ImgProc.h"
#include "cuda/ImgProcCuda.h"

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
		LaneDetector();
		~LaneDetector();

		void processFrame(cv::Mat & rgbMat, cv::Mat & dMat); // DO SOMETHING

		cv::Mat xyzMap,laneMap2D;
	private:
		ImgProc3D::Intr m_camInfo;
		IMAGE_SOURCE m_img_source;
		cv::Vec4f m_previous_plane_model;

		cv::cuda::GpuMat dev_dMat, dev_rgbMat;
		cv::cuda::GpuMat dev_xyzMap, dev_laneMap2D;

		void fillLaneMap();
		void detectLaneCenter();
	};
}


#endif