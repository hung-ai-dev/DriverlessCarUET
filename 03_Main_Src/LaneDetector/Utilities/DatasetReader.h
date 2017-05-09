#include <string>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#define USE_ASTRA
//#define USE_ASTRA_RAW
struct Intrinsics_TUM{
#ifdef USE_ASTRA
	float fx = 570.342165925f;	float fy = 570.341946943f;
	float cx = 319.5f;			float cy = 239.5f;
#ifdef USE_ASTRA_RAW
	float pixelToMeter = 1000.f;
#else
	float pixelToMeter = 5000.f;
#endif
	
#else
	float fx = 525.f;	float fy = 525.f; // Default of TUM
	float cx = 319.5f;	float cy = 239.5f;
	float pixelToMeter = 5000.f;
#endif // USE_ASTRA
};

//struct Intrinsics_ICL{
//	float fx = 481.2f;	float fy = -480.f;
//	float cx = 319.5f;	float cy = 239.5f;
//	float pixelToMeter = 5000.f;
//};

//struct Intrinsics_Astra{
//	float fx = 554.256258422f;	float fy = 520.600244485f;
//	float cx = 320;	float cy = 240;
//	float pixelToMeter = 5000.f;
//};

struct ImgHeader{
	double timeStamp;
	std::string filePath;
};

struct SensorHeader{
	double timeStamp;
	cv::Vec3f accl;
};

struct PoseParams{
	double timeStamp;
	double tx, ty, tz;
	double qx, qy, qz, qw;
};

struct FullFrame
{
	ImgHeader				depthHeader;
	ImgHeader				rgbHeaders;
	std::vector<SensorHeader>	accls;
	PoseParams				imgPose;
};

struct PointCloudXYZRGB
{
	std::vector<cv::Point3f> points;
	std::vector<cv::Vec3b> colors;
};

int readImageHeader(std::string fileName, std::vector<ImgHeader> & dHeader);
int readSensorHeader(std::string fileName, std::vector<SensorHeader> & sHeader);
int readGroundTruth(std::string fileName, std::vector<PoseParams> & sHeader);

int syncFrames(
	std::vector<ImgHeader> & depthFrames,
	std::vector<ImgHeader> & rgbFrames,
	std::vector<SensorHeader> & accl,
	std::vector<PoseParams> & poses,
	std::vector<FullFrame> & frames);

int convertToPointcloud(cv::Mat & rgbImg, cv::Mat & depthImg, Intrinsics_TUM cam, PointCloudXYZRGB & _cloud);

int readSyncFileHeader(std::string fileName, std::vector<FullFrame> & frame);
int readPoseFileICL(std::string fileName, std::vector<PoseParams> & frame);

template <typename T> void convertToVector(const cv::Mat & img, std::vector<T> & dataColor)
{
	int width = img.cols;
	dataColor.resize(img.rows * img.cols);
	for (int i = 0; i < img.rows; i++)	{
		const T * _d = img.ptr<T>(i);
		std::copy(_d, _d + width, dataColor.begin() + i*width);
	}
}

Eigen::Matrix4d getPose_ICL(const PoseParams & p);
Eigen::Matrix4d getPose_TUM(const PoseParams & p);
cv::Affine3d eigen2CV(Eigen::Matrix4d & tMat);