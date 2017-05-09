#include "opencv2/photo.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "stdio.h"
#include "iostream"

using namespace std;
using namespace cv;

Point getCenterPoint(Mat img) {  /// detect Orange point
	Mat hsvImg; 
	cvtColor(img, hsvImg, CV_BGR2HSV); /// rgb -> hsv
	Mat result;
	
	inRange(hsvImg, Scalar(0,130,130), Scalar(20,255,255), result); /// find Orange point -> grayscale
	//imshow("result", result);
	int numRow = result.rows;
	int numCol = result.cols;
	//cout << numRow << " " << numCol << endl;
	int sumRow = 0, sumCol = 0, cPixel = 0;
	int pre = -1;
	
		
	for (int i = numRow - 1; i >= 0; i--) {
		for (int j = 0; j < numCol; j++) {
			if ((int)result.at<uchar>(i,j) > 0) {
				if (pre == -1) pre = i;
				if (abs(i - pre) < 4) {
					pre = i;
					sumRow += i; sumCol += j;
					cPixel++;
					
				}

			}
		}
	}
	if (cPixel == 0) return Point(numCol / 2, numRow / 2);

	sumCol /= cPixel;
	sumRow /= cPixel;

	hsvImg.release();
	result.release();
	
	return Point(sumCol, sumRow);
}

int main() {
	for (int i = 0; i < 800; i++) {
		char str[10];
		sprintf(str, "%d", i);
		string tmp = str;
		string inName = "rgb/" + tmp + ".png";
		Mat img = imread(inName);
		//imshow("Input", img);
		Point center = getCenterPoint(img);
		circle(img, center, 5, Scalar(0, 255, 0), 5);
		string outName = "ttt/" + tmp + ".png";
		imwrite(outName, img);
		//imshow("Output", img);
		img.release();
		//char key = waitKey(-1);
		//if (key != 'a') break;
		//waitKey(0);
	}
}
