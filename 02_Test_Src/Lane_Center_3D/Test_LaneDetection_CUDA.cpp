#include "Utilities/DatasetReader.h"
#include "ImgProc/ImgProc.h"
#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/LaneDetector.h"

//#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

Intrinsics_TUM intrc;
int g_Width = 640;
int g_Height = 480;

std::vector<FullFrame> frames;
std::string dirPath = "D:/LANE_DATASET/DataCDS/Sample-03/";
cv::viz::Viz3d viz;
int idx = 0;

void setupViz3D();

void test_GridPlane_generated(cv::Mat & bgrMat, cv::Mat &dMat)
{
	int64 st = cv::getTickCount();
	ImgProc3D::Intr camInfo = ImgProc3D::Intr(5000.f);
	cv::Vec4f planeModel = fast_PlaneDetect(dMat, bgrMat, camInfo);
	cv::cuda::GpuMat dev_dMat(dMat);
	cv::cuda::GpuMat dev_rgbMat(bgrMat);
	//dev_dMat.upload(dMat);
	//dev_rgbMat.upload(bgrMat);
	cv::cuda::GpuMat dev_xyzMap(dMat.rows, dMat.cols, CV_32FC3);

	ImgProc3D::convertTo_Point3fMap(dev_dMat, camInfo, dev_xyzMap);

	cv::cuda::GpuMat dev_laneMap2D(LANE_MAP_SIZE, LANE_MAP_SIZE, CV_8UC3, cv::Scalar(128, 128, 128));
	ImgProc3D::genPlane2DMap(dev_xyzMap, dev_rgbMat, planeModel, dev_laneMap2D);

	cv::Mat xyzMap, laneMap2D;
	dev_laneMap2D.download(laneMap2D);

	printf("GPU time %f \n", (cv::getTickCount() - st) / cv::getTickFrequency());

	cv::imshow("RGB", bgrMat);
	cv::imshow("DEPTH", dMat);
	cv::imshow("LANE", laneMap2D);
	cv::waitKey(5);
}


int main()
{
	//cv::Mat bgrMat = cv::imread("../TestData/A-RGB-55.png");
	//cv::Mat dMat = cv::imread("../TestData/A-D-55.png", CV_LOAD_IMAGE_ANYDEPTH);
	ImgProc3D::LaneDetector lane_detector;

	readSyncFileHeader(dirPath + "associations.txt", frames);
	setupViz3D();

	while (idx < frames.size())
	{
		cv::Mat bgrMat = cv::imread(dirPath + frames[idx].rgbHeaders.filePath);
		cv::Mat dMat = cv::imread(dirPath + frames[idx].depthHeader.filePath, CV_LOAD_IMAGE_ANYDEPTH);
		//test_GridPlane_generated(bgrMat, dMat);

		lane_detector.processFrame(bgrMat, dMat);

		cv::Mat gray_img;
		cv::cvtColor(lane_detector.laneMap2D, gray_img, cv::COLOR_BGR2GRAY);
		std::vector<cv::Point3f> centers = find_lane_center(gray_img, 65);

		for (int i = 0; i < centers.size(); i++)
		{
			cv::circle(lane_detector.laneMap2D, cv::Point2f(centers[i].x, 512-centers[i].y), 3, cv::Scalar(100, 250, 0), 2);
		}

		cv::imshow("LANE", lane_detector.laneMap2D);
		cv::waitKey(5);
		//DO SOMETHING:
		idx++;

		//viz.spinOnce();
		printf("frame no %d / %d \n", idx, frames.size());
	}

	//setupViz3D();
	//viz.spin();
	return 0;
}

void setupViz3D()
{
	viz = cv::viz::Viz3d("show_cloud");
	viz.showWidget("coo-sys", cv::viz::WCoordinateSystem());

	cv::Vec3f cam_pos(0.0f, 0.0f, 0.0f), cam_focal_point(0.0f, 0.0f, 2.0f), cam_y_dir(0.0f, 1.0f, 0.0f);
	cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
	viz.setViewerPose(cam_pose);
}
