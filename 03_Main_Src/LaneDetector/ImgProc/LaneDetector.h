#ifndef __LANE_DETECTOR_H
#define __LANE_DETECTOR_H

#include "ImgProc.h"
#include "cuda/ImgProcCuda.h"

#include <cmath>
#include <fstream>

#define MaskSize 9

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
		void calibFrame(cv::Mat & rgbMat, cv::Mat & dMat);
		void processFrame(cv::Mat & rgbMat, cv::Mat & dMat); // DO SOMETHING

		void findLane(const cv::Mat& image) ;
		bool detectLaneCenter(std::vector<cv::Point2f> centers, cv::Point2f& center);
		std::vector<cv::Point2f> findLaneCenter(const cv::Mat& image, int radius = 65,
									bool useLeftLane = 0, bool useRightLane = 0); 

		cv::Point projectToMap2D(const cv::Mat & dMat, cv::Point & left, cv::Point & right);
		int checkObsPosition(std::vector<cv::Point> left, std::vector<cv::Point> right, cv::Point obs);

		ImgProc3D::Intr getCamInfor();

		cv::Mat xyzMap,laneMap2D, objectMask;	
		bool smallImage;
		cv::Vec4f planeModel;

		cv::Point iniLeft, iniRight;
		std::vector<cv::Point> left_line, right_line; 
		std::vector<cv::Point> iniLefts, iniRts;

		cv::gpu::GpuMat dev_dMat, dev_rgbMat, dev_xyzMap, dev_laneMap2D, dev_objectMask;
	private:
		ImgProc3D::Intr m_camInfo;
		IMAGE_SOURCE m_img_source;
		cv::Vec4f m_previous_plane_model;
		cv::Mat laneProject2D;

		void fillLaneMap();

		void copyPoint(cv::Point src, cv::Point& dst);

		void findInitialPoint(const cv::Mat& image, cv::Point& pLeft, cv::Point& pRight,
     		bool& flagLeft, bool& flagRight, int offsetX, int offsetY);

		void lineProcessing(const cv::Mat& image, cv::Point first, int flag, std::vector<cv::Point>& line, 
    		int off_set_x, int off_set_y);

		std::vector<cv::Point> seederPostProcessing(std::vector<cv::Point> seeder);
	};
}


#endif