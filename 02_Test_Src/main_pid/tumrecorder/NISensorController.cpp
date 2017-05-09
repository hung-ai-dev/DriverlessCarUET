#include "NISensorController.h"
#define RECORD_PATH			"C:\\Users\\HoangQC\\Desktop\\KinectDB\\record.oni"
#define ONI_FILE_PATH		"C:\\Users\\HoangQC\\Desktop\\KinectDB\\convert.oni"


NISensorController::NISensorController()
{

}


NISensorController::~NISensorController()
{
	recordStop();
	closeKinect();
}

int NISensorController::initKinect()
{
	openni::Status rc = openni::STATUS_OK;
	const char* deviceURI = openni::ANY_DEVICE;

	rc = openni::OpenNI::initialize();
	printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

	openni::VideoMode depthVM,colorVM;
	
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
	if (!HandleStatus(rc))			{   depthStream.destroy();return  openni::STATUS_ERROR;   }

	rc = colorStream.create(*p_device, openni::SENSOR_COLOR);
	rc = colorStream.setVideoMode(colorVM);
	rc = colorStream.start();
	if (!HandleStatus(rc))			{   colorStream.destroy();return  openni::STATUS_ERROR;   }
	
	getSensorInfo(depthStream);
	getSensorInfo(colorStream);

	colorStream.setMirroringEnabled(false);
	depthStream.setMirroringEnabled(false);
	
	if (!depthStream.isValid() || !colorStream.isValid())
	{
		printf("No valid streams. Exiting\n");
		openni::OpenNI::shutdown();
		return 2;
	}
	colorStream.addNewFrameListener(this);

	p_device->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	p_device->setDepthColorSyncEnabled(true);
	
	hasNewframe = false;
	return openni::STATUS_OK;
}

void NISensorController::closeKinect()
{
	colorStream.removeNewFrameListener(this);
	depthStream.destroy();
	colorStream.destroy();
	p_device->close();
	openni::OpenNI::shutdown();
}

void NISensorController::recordStart()
{
	if (!m_recorder.isValid())
	{
		m_recorder.create(RECORD_PATH);
		m_recorder.attach(depthStream);
		m_recorder.attach(colorStream);
		m_recorder.start();
	}
	
}
void NISensorController::recordStop()
{
	if (m_recorder.isValid())
	{
		m_recorder.stop();
		m_recorder.destroy();
	}
}
/* 
Update Color Event 
*/
void NISensorController::onNewFrame(openni::VideoStream & videoStream)
{
	frame_mutex.lock();
	colorStream.readFrame(&m_colorFrame);
	pColor = (uint8_t *)m_colorFrame.getData();
	uint64_t c_time = m_colorFrame.getTimestamp();

	depthStream.readFrame(&m_depthFrame);
	pDepth = (openni::DepthPixel *)m_depthFrame.getData();
	uint64_t d_time = m_depthFrame.getTimestamp();

	hasNewframe = true;
	frame_mutex.unlock();
	//std::cout << "new-frame at: " << c_time << std::endl;

	/*cv::Mat colorRaw = cv::Mat(480, 640, CV_8UC3, pColor);
	cv::imwrite("CL.png", colorRaw);*/
}
