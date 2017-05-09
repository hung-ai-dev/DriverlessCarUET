#include "ImgProc/ImgProc.h"

#include "opencv2/opencv.hpp"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "opencv2/gpu/gpu.hpp"

int main()
{
	cv::Mat bgrMat = cv::imread("../TestData/RGB-0.png");
	cv::Mat dMat = cv::imread("../TestData/D-0.png", CV_LOAD_IMAGE_ANYDEPTH);

	ImgProc3D::Intr m_camInfo(ImgProc3D::IntrMode_320x240_RAW);

	cv::Vec4f planeModel = fast_PlaneDetect(dMat, bgrMat, m_camInfo, true);

	cv::gpu::GpuMat dev_dMat(dMat);
	cv::gpu::GpuMat dev_rgbMat(bgrMat);


	cv::gpu::GpuMat dev_xyzMap(dMat.rows, dMat.cols, CV_32FC3);
	cv::gpu::GpuMat dev_normalMap(dMat.rows, dMat.cols, CV_32FC3);
	cv::gpu::GpuMat dev_objectMap(dMat.rows, dMat.cols, CV_8UC1);
	ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);
	ImgProc3D::convertTo_NormalsMap(dev_xyzMap, dev_normalMap);

	cv::gpu::GpuMat dev_laneMap2D(LANE_MAP_SIZE, LANE_MAP_SIZE, CV_8UC3, cv::Scalar(128, 128, 128));
	ImgProc3D::genPlane2DMap(dev_xyzMap, dev_rgbMat, planeModel, dev_laneMap2D, dev_objectMap);

	cv::Mat xyzMap, laneMap2D, normalMap, objMap;
	dev_laneMap2D.download(laneMap2D);
	dev_normalMap.download(normalMap);
	dev_objectMap.download(objMap);
	cv::imshow("RGB", bgrMat);
	cv::imshow("DEPTH", dMat);
	cv::imshow("LANE", laneMap2D);
	cv::imshow("NORMAL", normalMap);
	cv::imshow("OBJ", objMap);
	cv::waitKey();

	return 0;
}