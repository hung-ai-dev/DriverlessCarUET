//#pragma warning(disable:4996)

#include "Utilities/DatasetReader.h"
#include <stdint.h>

using namespace std;
vector<ImgHeader> depthHeader, rgbHeader;
vector<SensorHeader> ssHeader;
vector<PoseParams> camPoses;

vector<FullFrame> frames;

std::string dirPath = "/media/hungnd/Data/LANE_DATASET/DataCDS/Sample-03/";

int main(int argc, char** argv)
{
	readSyncFileHeader(dirPath + "associations.txt", frames);
	Intrinsics_TUM camInfo;

	cv::viz::Viz3d viz("show_cloud");
	viz.showWidget("coo-sys", cv::viz::WCoordinateSystem());

	/// Let's assume camera has the following properties
	cv::Vec3f cam_pos(0.0f, 0.0f, -1.0f), cam_focal_point(0.0f, 0.0f, 2.0f), cam_y_dir(0.0f, 1.0f, 0.0f);
	cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
	viz.setViewerPose(cam_pose);

	int idx = 0;
	while (idx < frames.size())
	{
		FullFrame var = frames[idx];
		cv::Mat img = cv::imread(dirPath + var.rgbHeaders.filePath);
		cv::Mat dimg = cv::imread(dirPath + var.depthHeader.filePath, CV_LOAD_IMAGE_ANYDEPTH);

		PointCloudXYZRGB cloudXYZrgb;
		convertToPointcloud(img, dimg, camInfo, cloudXYZrgb);

		//cv::Affine3d cvaff = eigen2CV(getPose_TUM(var.imgPose));

		viz.showWidget("w", cv::viz::WCloud(cloudXYZrgb.points, cloudXYZrgb.colors)/*, cvaff*/);
		viz.spinOnce(1, true);

		idx++;

		cv::imshow("rgb", img); cv::imshow("d", dimg);
		if (cv::waitKey(5) == 'q') break;
	}

	return 0;
}

