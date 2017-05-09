#include "LaneDetector.h"

ImgProc3D::LaneDetector::LaneDetector() :m_camInfo(5000.f)
{
	int imgWidth = 640;
	int imgHeight = 480;
	dev_dMat = cv::cuda::GpuMat(imgHeight,imgWidth,CV_16UC1);
	dev_rgbMat = cv::cuda::GpuMat(imgHeight, imgWidth, CV_8UC3);
	dev_xyzMap = cv::cuda::GpuMat(imgHeight, imgWidth, CV_32FC3);
	dev_laneMap2D = cv::cuda::GpuMat(LANE_MAP_SIZE, LANE_MAP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0));
}

ImgProc3D::LaneDetector::~LaneDetector()
{
	
}

void ImgProc3D::LaneDetector::processFrame(cv::Mat & rgbMat, cv::Mat & dMat)
{
	cv::Vec4f planeModel = fast_PlaneDetect(dMat, rgbMat, m_camInfo);
	
	if (planeModel[1] == 1.0f)	{
		planeModel = m_previous_plane_model;
	}	else	{
		m_previous_plane_model = planeModel;
	}
	
	dev_dMat.upload(dMat);
	dev_rgbMat.upload(rgbMat);

	ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);

	dev_laneMap2D.setTo(cv::Scalar(128, 128, 128));
	ImgProc3D::genPlane2DMap(dev_xyzMap, dev_rgbMat, planeModel, dev_laneMap2D);

	dev_laneMap2D.download(laneMap2D);
	//
}