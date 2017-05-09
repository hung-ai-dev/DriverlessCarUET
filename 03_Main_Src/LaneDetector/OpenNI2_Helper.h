#include "OpenNI.h"

/*Utility*/
char ReadLastCharOfLine();
bool HandleStatus(openni::Status status);
void getSensorInfo(openni::VideoStream & stream);

int setupAstra(openni::Device * p_device, 
	openni::VideoStream & depthStream, 
	openni::VideoStream & colorStream, 
	int raw_width = 320, int raw_height = 240, int fps = 30)
{
	openni::Status rc = openni::STATUS_OK;
	const char* deviceURI = openni::ANY_DEVICE;

	rc = openni::OpenNI::initialize();
	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	openni::VideoMode depthVM, colorVM;

	depthVM.setFps(fps);
	depthVM.setResolution(raw_width, raw_height);
	depthVM.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);

	colorVM.setFps(fps);
	colorVM.setResolution(raw_width, raw_height);
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