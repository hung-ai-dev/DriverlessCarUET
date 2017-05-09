#include "Utilities/DatasetReader.h"
#include <Eigen/Core>
#include "msac/api_lane_detection.h"

Intrinsics_TUM intrc;
MSAC msac;
int g_Width = 640;
int g_Height = 480;

void fastPlaneEstimate(cv::Mat & xyzMat);

int detectLines(cv::Mat & imgInput)
{
	cv::Mat imgGray;

	
	cv::Rect roi = cv::Rect(0, imgInput.rows/2, imgInput.cols, imgInput.rows/2);

	char key = 0;

	int64 st = 0, et = 0, fps = 0;
	double freq = cv::getTickFrequency();

	st = cv::getTickCount();
	vector< Point > points;
	int max_size = 5;

	// Convert to gray
	cv::cvtColor(imgInput, imgGray, CV_BGR2GRAY);
	Point vp;
	vector<vector<cv::Point> > lineSegments;

	api_get_vanishing_point(imgGray, roi, msac, vp, true, "Canny");
	points.push_back(vp);

	for (int i = 0; i < points.size(); i++)
	{
		// std::cout << points[i].x << " - " << points[i].y << std::endl;
		cv::circle(imgInput, points[i], 5, cv::Scalar(0, 0, 0), 4);
	}

	et = cv::getTickCount();
	printf("time = %f \n", (et - st) / getTickFrequency());
	imshow("Debug", imgInput);
	cv::waitKey(5);
	return 0;
}

vector<FullFrame> frames;
std::string dirPath = "D:/LANE_DATASET/DataCDS/Sample-04/";

int main()
{
	readSyncFileHeader(dirPath + "associations.txt", frames);

	api_vanishing_point_init(msac);
	int idx = 0;

	while (idx < frames.size())
	{
		FullFrame var = frames[idx];
		cv::Mat img = cv::imread(dirPath + var.rgbHeaders.filePath);
		detectLines(img);
		idx++;
	}
	return 0;
}