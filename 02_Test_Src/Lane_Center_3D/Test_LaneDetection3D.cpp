#include "Utilities/DatasetReader.h"
#include "ImgProc/ImgProc.h"
#include <Eigen/Dense>

Intrinsics_TUM intrc;
int g_Width = 640;
int g_Height = 480;

std::vector<FullFrame> frames;
std::string dirPath = "/media/hungnd/Data/LANE_DATASET/DataCDS/Sample-03/";
cv::viz::Viz3d viz;
int idx = 0;

void setupViz3D()
{
	viz = cv::viz::Viz3d("show_cloud");
	viz.showWidget("coo-sys", cv::viz::WCoordinateSystem());

	cv::Vec3f cam_pos(0.0f, 0.0f, 0.0f), cam_focal_point(0.0f, 0.0f, 2.0f), cam_y_dir(0.0f, 1.0f, 0.0f);
	cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
	viz.setViewerPose(cam_pose);
}

int test_Plane_generated(cv::Mat & bgrMat, cv::Mat &dMat);
int test_GridPlane_generated(cv::Mat & bgrMat, cv::Mat &dMat);

int main2()
{
	setupViz3D();

	cv::Mat bgrMat = cv::imread("../TestData/A-RGB-55.png");
	cv::Mat dMat = cv::imread("../TestData/A-D-55.png", CV_LOAD_IMAGE_ANYDEPTH);

	//test_Plane_generated(bgrMat, dMat);
	Intrinsics_TUM camInfo;
	PointCloudXYZRGB cloudXYZrgb;
	convertToPointcloud(bgrMat, dMat, camInfo, cloudXYZrgb);
	cv::Mat xyzMat = cv::Mat(cv::Size(g_Width, g_Height), CV_32FC3, cloudXYZrgb.points.data());

	ImgProc3D::Intr camAstra(5000.f);
	cv::Vec4f plane = fast_PlaneDetect(dMat, bgrMat, camAstra);

	cv::Mat lane2DMap(xyzMat.rows, xyzMat.cols, CV_32FC2, cv::Scalar(0, 0));
	cv::Mat lane2DGridMap(512, 512, CV_8UC3, cv::Scalar(128,128,128));
	fillLaneMap2D(xyzMat, lane2DMap, plane, 0.05f);
	create2DGrid(lane2DMap, bgrMat, lane2DGridMap);
	//TODO: from 2D map find center points 
	// ...

	cv::imshow("RGB", bgrMat);
	cv::imshow("DEPTH", dMat);
	cv::imshow("LANE-GRID", lane2DGridMap);

	//cv::imwrite("../TestData/A-Grid2D-55.png", lane2DGridMap);

	viz.showWidget("W", cv::viz::WCloud(cloudXYZrgb.points, cloudXYZrgb.colors));
	viz.showWidget("L", cv::viz::WArrow(cv::Point3f(0, 0, 0), cv::Point3f(plane[0], plane[1], plane[2])));

	viz.spin();
	return 0;
}

int main()
{
	readSyncFileHeader(dirPath + "associations.txt", frames);

	setupViz3D();

	while (idx < frames.size())
	{
		cv::Mat bgrMat = cv::imread(dirPath + frames[idx].rgbHeaders.filePath);
		cv::Mat dMat = cv::imread(dirPath + frames[idx].depthHeader.filePath, CV_LOAD_IMAGE_ANYDEPTH);
		test_GridPlane_generated(bgrMat, dMat);		

		//DO SOMETHING:
		idx++;

		viz.spinOnce();
		printf("frame no %d / %d \n", idx, frames.size());
	}
	return 0;
}

int test_Plane_generated(cv::Mat & bgrMat, cv::Mat &dMat)
{
	Intrinsics_TUM camInfo;
	PointCloudXYZRGB cloudXYZrgb;
	convertToPointcloud(bgrMat, dMat, camInfo, cloudXYZrgb);
	cv::Mat xyzMat = cv::Mat(cv::Size(g_Width, g_Height), CV_32FC3, cloudXYZrgb.points.data());

	ImgProc3D::Intr camAstra = ImgProc3D::Intr(5000.f);
	cv::Vec4f plane = fast_PlaneDetect(dMat, bgrMat, camAstra);

	//cv::Mat lane2DMap;
	//fillLaneMap2D(xyzMat, lane2DMap, plane, 0.05f);

	cv::Point3f planeNormal(plane[0], plane[1], plane[2]);
	cv::Point3f e_1 = planeNormal.cross(cv::Point3f(0, 0, 1));
	cv::Point3f e_2 = -planeNormal.cross(e_1);
	cv::Point3f planeOrg = -plane[3] * planeNormal;
	std::vector<cv::Point3f> laneXY;
	std::vector<cv::Vec3b> laneXY_color;

	for (int i = 0; i < bgrMat.rows; i++)
	{
		for (int j = 0; j < bgrMat.cols; j++)
		{
			cv::Point3f p = xyzMat.at<cv::Point3f>(i, j);
			if (!std::isnan(p.x))
			{
				if (fabs(plane[0] * p.x + plane[1] * p.y + plane[2] * p.z + plane[3]) < 0.05f)
				{

					cv::Point3f p_t = p - planeOrg;

					float p_x_new = p_t.dot(e_1);
					float p_y_new = p_t.dot(e_2);
					laneXY.push_back(cv::Point3f(p_x_new, p_y_new, 0));

					cv::Vec3b clr = bgrMat.at<cv::Vec3b>(i, j);
					laneXY_color.push_back(cv::Vec3b(clr[0], 200, clr[2]));

					bgrMat.at<cv::Vec3b>(i, j)[1] = 200;
				}
				else{
					bgrMat.at<cv::Vec3b>(i, j)[2] = 200;
				}
			}

		}
	}

	cv::imshow("RGB", bgrMat);
	cv::imshow("DEPTH", dMat);

	viz.showWidget("W", cv::viz::WCloud(cloudXYZrgb.points, cloudXYZrgb.colors));
	viz.showWidget("L", cv::viz::WArrow(cv::Point3f(0, 0, 0), cv::Point3f(plane[0], plane[1], plane[2])));

	//Debug Plane2D Lane
	viz.showWidget("Plane2D", cv::viz::WCloud(laneXY, laneXY_color));
	viz.showWidget("L-x", cv::viz::WLine(cv::Point3f(0, 0, 0), e_1 * 5, cv::viz::Color::red()));
	viz.showWidget("L-y", cv::viz::WLine(cv::Point3f(0, 0, 0), e_2*2.5, cv::viz::Color::green()));
	viz.showWidget("C", cv::viz::WSphere(planeOrg, 0.05, 10, cv::viz::Color::green()));

	return 0;
}

int test_GridPlane_generated(cv::Mat & bgrMat, cv::Mat &dMat)
{
	int64 st = cv::getTickCount();
	Intrinsics_TUM camInfo;
	PointCloudXYZRGB cloudXYZrgb;
	convertToPointcloud(bgrMat, dMat, camInfo, cloudXYZrgb);
	cv::Mat xyzMat = cv::Mat(cv::Size(g_Width, g_Height), CV_32FC3, cloudXYZrgb.points.data());

	ImgProc3D::Intr camAstra(5000.f);
	cv::Vec4f plane = fast_PlaneDetect(dMat, bgrMat, camAstra);

	cv::Mat lane2DMap(xyzMat.rows, xyzMat.cols, CV_32FC2, cv::Scalar(0, 0));
	cv::Mat lane2DGridMap(512, 512, CV_8UC3, cv::Scalar(128, 128, 128));
	fillLaneMap2D(xyzMat, lane2DMap, plane, 0.05f);
	create2DGrid(lane2DMap, bgrMat, lane2DGridMap);
	// ...
	// cv::imwrite(dirPath + "lane/" + std::to_string(idx) + ".png", lane2DGridMap);
	printf("GPU time %f \n", (cv::getTickCount() - st) / cv::getTickFrequency());
	cv::imshow("RGB", bgrMat);
	cv::imshow("DEPTH", dMat);
	cv::imshow("LANE-GRID", lane2DGridMap);

	//cv::imwrite("../TestData/A-Grid2D-55.png", lane2DGridMap);

	viz.showWidget("W", cv::viz::WCloud(cloudXYZrgb.points, cloudXYZrgb.colors));
	viz.showWidget("L", cv::viz::WArrow(cv::Point3f(0, 0, 0), cv::Point3f(plane[0], plane[1], plane[2])));

	return 0;
}
