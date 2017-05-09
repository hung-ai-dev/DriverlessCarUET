#include "opencv2/opencv.hpp"
// #include <conio.h>

using namespace cv;
int main()
{
	KalmanFilter KF(1, 1, 0);
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(0.02));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	// Important parameters
	setIdentity(KF.errorCovPost, Scalar::all(0.1));

	Mat measurement = Mat(1, 1, CV_32F,cv::Scalar(0));
	Mat kfState(1, 1, CV_32F, cv::Scalar(-6));
	Mat correctMat(1, 1, CV_32F, cv::Scalar(0));
	//Set state
	KF.statePost = kfState;
	Mat prediction = KF.predict();
	//Mat predict = Mat::zeros(1, 1, CV_32F);
	//predict = KF.predict(); std::cout << predict << std::endl;
	
	measurement.at<float>(0) = -3;
	std::cout << KF.correct(measurement) << std::endl;

	measurement.at<float>(0) = 0;
	std::cout << KF.correct(measurement) << std::endl;

	measurement.at<float>(0) = 3;
	std::cout << KF.correct(measurement) << std::endl;

	measurement.at<float>(0) = 6;
	std::cout << KF.correct(measurement) << std::endl;

	//std::cout << KF.predict() << std::endl;

	//KF.statePost = measurement;
	//predict = KF.predict(); std::cout << predict << std::endl;
	// _getch();
}