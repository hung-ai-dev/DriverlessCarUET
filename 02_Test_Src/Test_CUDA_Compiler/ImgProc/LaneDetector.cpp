#include "LaneDetector.h"

#ifndef LANE_MAP_SIZE
#define LANE_MAP_SIZE 512
#endif

static float qnan = std::numeric_limits<float>::quiet_NaN();

ImgProc3D::LaneDetector::LaneDetector(int imgWidth, int imgHeight) :m_camInfo(5000.f)
{
	smallImage = false;
	xyzMap = cv::Mat(imgHeight, imgWidth, CV_32FC3);
	laneProject2D = cv::Mat(imgHeight, imgWidth, CV_32FC2, cv::Scalar(qnan, qnan));
	laneMap2D = cv::Mat(LANE_MAP_SIZE, LANE_MAP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0));
	/*dev_dMat = cv::cuda::GpuMat(imgHeight,imgWidth,CV_16UC1);
	dev_rgbMat = cv::cuda::GpuMat(imgHeight, imgWidth, CV_8UC3);
	dev_xyzMap = cv::cuda::GpuMat(imgHeight, imgWidth, CV_32FC3);
	dev_laneMap2D = cv::cuda::GpuMat(LANE_MAP_SIZE, LANE_MAP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0));*/
}

ImgProc3D::LaneDetector::~LaneDetector()
{
	
}

void ImgProc3D::LaneDetector::setCamera(ImgProc3D::IntrMode _mode)
{
	m_camInfo = ImgProc3D::Intr(_mode);
	if (_mode == ImgProc3D::IntrMode_320x240_IMG || _mode == ImgProc3D::IntrMode_320x240_RAW) {	smallImage = true; }
	else {	smallImage = false;	}
}

void ImgProc3D::LaneDetector::processFrame(cv::Mat & rgbMat, cv::Mat & dMat)
{
	cv::Vec4f planeModel = fast_PlaneDetect(dMat, rgbMat, m_camInfo, smallImage);
	
	if (planeModel[1] == 1.0f)	{
		planeModel = m_previous_plane_model;
	}	else	{
		m_previous_plane_model = planeModel;
	}
	
	convertToXYZ(dMat, m_camInfo, xyzMap);

	laneProject2D = cv::Scalar(qnan, qnan);
	fillLaneMap2D(xyzMap, laneProject2D, planeModel);
	
	laneMap2D = cv::Scalar(100, 100, 100);
	create2DGrid(laneProject2D, rgbMat, laneMap2D);

}