#ifndef __NI_SENSOR__
#define __NI_SENSOR__
#include "OpenNI.h"
#include "opencv2/opencv.hpp"	
#include <mutex>

#define RAW_WIDTH	320
#define RAW_HEIGHT	240

class NISensorController : public openni::VideoStream::NewFrameListener
{
public:
	NISensorController();
	~NISensorController();

	/*main function*/
	int initKinect();
	void closeKinect();
	void onNewFrame(openni::VideoStream & videoStream);

	/*record*/
	void recordStart();
	void recordStop();

	openni::Device				* p_device;
	openni::VideoStream			depthStream, colorStream;

	unsigned char				* pColor;
	openni::DepthPixel			* pDepth;
	bool hasNewframe;
private:
	openni::VideoFrameRef		m_depthFrame, m_colorFrame;
	openni::Recorder			m_recorder;
	std::mutex frame_mutex;
};

/*Utility*/
inline char ReadLastCharOfLine()
{
	int newChar = 0;
	int lastChar;
	fflush(stdout);
	do
	{
		lastChar = newChar;
		newChar = getchar();
	}
	while ((newChar != '\n') && (newChar != EOF));
	return (char)lastChar;
} 
inline bool HandleStatus(openni::Status status)
{
	if (status == openni::STATUS_OK)
		return true;
	printf("ERROR: #%d, %s", status,
		openni::OpenNI::getExtendedError());
	ReadLastCharOfLine();
	return false;
}
inline void getSensorInfo(openni::VideoStream & stream)
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

#endif
