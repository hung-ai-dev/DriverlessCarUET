#include "ImgProc/ImgProc.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
//#include "ImgProc/cuda/ImgProcCuda.h"
#include "ImgProc/LaneDetector.h"

#include "OpenNI.h"
#include <sys/select.h>

#define RAW_WIDTH 640
#define RAW_HEIGHT 480
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000 ms

openni::Device				* p_device;
int setupAstra(openni::VideoStream	& depthStream, openni::VideoStream & colorStream);

/*Utility*/
char ReadLastCharOfLine();
bool HandleStatus(openni::Status status);
void getSensorInfo(openni::VideoStream & stream);

int _kbhit() {
 	struct timeval tv;
  	fd_set read_fd;

  	/* Do not wait at all, not even a microsecond */
  	tv.tv_sec=0;
  	tv.tv_usec=0;

  	/* Must be done first to initialize read_fd */
  	FD_ZERO(&read_fd);

  	/* Makes select() ask if input is ready:
   	* 0 is the file descriptor for stdin    */
  	FD_SET(0,&read_fd);

  	/* The first parameter is the number of the
   	* largest file descriptor to check + 1. */
  	if(select(1, &read_fd,NULL, /*No writes*/NULL, /*No exceptions*/&tv) == -1)
    return 0;  /* An error occured */

  	/*  read_fd now holds a bit map of files that are
   	* readable. We test the entry for the standard
   	* input (file 0). */
  
	if(FD_ISSET(0,&read_fd))
    /* Character pending on stdin */
    return 1;

  	/* no characters were pending */
  	return 0;
}

int main()
{
	openni::VideoStream	depthStream, colorStream;
	openni::VideoFrameRef		m_depthFrame, m_colorFrame;
	
	openni::VideoStream* streams[] = { &depthStream, &colorStream };
	setupAstra(depthStream, colorStream);

	ImgProc3D::LaneDetector lane_detector;

	while (!_kbhit())
	{
		int changedStreamDummy;
		char rc = openni::OpenNI::waitForAnyStream(streams, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
		if (rc != openni::STATUS_OK){
			printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, openni::OpenNI::getExtendedError());
			continue;
		}
		if (rc != openni::STATUS_OK){
			printf("Read failed!\n%s\n", openni::OpenNI::getExtendedError());
			continue;
		}
		colorStream.readFrame(&m_colorFrame);
		depthStream.readFrame(&m_depthFrame);

		cv::Mat depthMat = cv::Mat(RAW_HEIGHT, RAW_WIDTH, CV_16UC1, (openni::DepthPixel *)m_depthFrame.getData());
		depthMat *= 5;

		cv::Mat bgrMat;
		cv::Mat colorRaw = cv::Mat(RAW_HEIGHT, RAW_WIDTH, CV_8UC3, (uint8_t *)m_colorFrame.getData());
		cv::cvtColor(colorRaw, bgrMat, CV_RGB2BGR);

		lane_detector.processFrame(bgrMat, depthMat);

		cv::Mat gray_img;
		cv::cvtColor(lane_detector.laneMap2D, gray_img, cv::COLOR_BGR2GRAY);
		std::vector<cv::Point3f> centers = find_lane_center(gray_img, 65);

		for (int i = 0; i < centers.size(); i++)
		{
			cv::circle(lane_detector.laneMap2D, cv::Point2f(centers[i].x, 512 - centers[i].y), 3, cv::Scalar(100, 250, 0), 2);
		}

		cv::imshow("LANE", lane_detector.laneMap2D);
		//cv::waitKey(5);

		cv::imshow("cl", bgrMat);
		cv::imshow("d", depthMat);
		cv::waitKey(5);
	}

	depthStream.stop();
	depthStream.destroy();
	colorStream.stop();
	colorStream.destroy();

	openni::OpenNI::shutdown();
	return 0;
}

int setupAstra(openni::VideoStream	& depthStream, openni::VideoStream & colorStream)
{
	openni::Status rc = openni::STATUS_OK;
	const char* deviceURI = openni::ANY_DEVICE;

	rc = openni::OpenNI::initialize();
	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	openni::VideoMode depthVM, colorVM;

	depthVM.setFps(30);
	depthVM.setResolution(RAW_WIDTH, RAW_HEIGHT);
	depthVM.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);

	colorVM.setFps(30);
	colorVM.setResolution(RAW_WIDTH, RAW_HEIGHT);
	colorVM.setPixelFormat(openni::PIXEL_FORMAT_RGB888);

	p_device = new openni::Device;
	rc = p_device->open(deviceURI);

	if (!HandleStatus(rc)) return openni::STATUS_ERROR;

	rc = depthStream.create(*p_device, openni::SENSOR_DEPTH);
	rc = depthStream.setVideoMode(depthVM);
	rc = depthStream.start();
	if (!HandleStatus(rc))			{ depthStream.destroy(); return  openni::STATUS_ERROR; }

	rc = colorStream.create(*p_device, openni::SENSOR_COLOR);
	rc = colorStream.setVideoMode(colorVM);
	rc = colorStream.start();
	if (!HandleStatus(rc))			{ colorStream.destroy(); return  openni::STATUS_ERROR; }

	getSensorInfo(depthStream);
	getSensorInfo(colorStream);

	colorStream.setMirroringEnabled(false);
	depthStream.setMirroringEnabled(false);

	if (!depthStream.isValid() || !colorStream.isValid())
	{
		printf("No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
	}

	p_device->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	p_device->setDepthColorSyncEnabled(true);

	return 0;
}

char ReadLastCharOfLine()
{
	int newChar = 0;
	int lastChar;
	fflush(stdout);
	do
	{
		lastChar = newChar;
		newChar = getchar();
	} while ((newChar != '\n') && (newChar != EOF));
	return (char)lastChar;
}

bool HandleStatus(openni::Status status)
{
	if (status == openni::STATUS_OK)
		return true;
	printf("ERROR: #%d, %s", status,
		openni::OpenNI::getExtendedError());
	ReadLastCharOfLine();
	return false;
}

void getSensorInfo(openni::VideoStream & stream)
{
	printf("Retrieving list of possible video modes for this stream ...\r\n");
	const openni::Array<openni::VideoMode> *supportedVideoModes = &(stream.getSensorInfo().getSupportedVideoModes());
	int numOfVideoModes = supportedVideoModes->getSize();

	for (int i = 0; i < numOfVideoModes; i++)
	{
		openni::VideoMode vm = (*supportedVideoModes)[i];
		printf("%c. %dx%d at %d fps with %d format \r\n",
			'a' + i,
			vm.getResolutionX(),
			vm.getResolutionY(),
			vm.getFps(),
			vm.getPixelFormat());
	}

	printf("Completed.\r\n");
}