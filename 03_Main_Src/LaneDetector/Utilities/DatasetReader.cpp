#include "DatasetReader.h"

//using namespace std;

int syncFrames(std::vector<ImgHeader> & depthFrames,
	std::vector<ImgHeader> & rgbFrames,
	std::vector<SensorHeader> & accl,
	std::vector<PoseParams> & poses,
	std::vector<FullFrame> & sync_Frames)
{
	int rgbInd = 0;
	int depInd = 0;
	int accInd = 0;
	int poseInd = 0;
	for(auto var : depthFrames)
	{
		FullFrame frame;
		frame.depthHeader = var;
		int rgbFound = -1;

		while (rgbFrames[rgbInd].timeStamp < frame.depthHeader.timeStamp + 0.01)
		{
			frame.rgbHeaders = rgbFrames[rgbInd];
			rgbFound = rgbInd;
			rgbInd++;
			if (rgbInd == rgbFrames.size()) return 0;
		}

		while (poses[poseInd].timeStamp < frame.depthHeader.timeStamp + 0.01)
		{
			frame.imgPose = poses[poseInd];
			poseInd++;
			if (poseInd == poses.size()) return 0;
		}

		while (accl[accInd].timeStamp < frame.depthHeader.timeStamp && accInd < accl.size())
		{
			frame.accls.push_back(accl[accInd]);	accInd++;
		}

		if (rgbFound != -1)
		{
			sync_Frames.push_back(frame);
			std::cout << "-Depth: " << depInd << " Color: " << rgbFound << " POSE: " << poseInd << std::endl;
		}

		depInd++;

		if (rgbInd == rgbFrames.size() || depInd == depthFrames.size())
		{
		}

	}

	return 0;
}

int readImageHeader(std::string fileName, std::vector<ImgHeader> & dHeader)
{
	std::ifstream headerFile;	headerFile.open(fileName.c_str());
	if (!headerFile.is_open()){ std::cout << "Could not open file\n"; return -1; }
	std::string temp;
	std::getline(headerFile, temp); std::cout << temp << std::endl;
	std::getline(headerFile, temp); std::cout << temp << std::endl;
	std::getline(headerFile, temp); std::cout << temp << std::endl;

	int line = 1;
	std::cout << std::fixed;
	while (!headerFile.eof())	{
		ImgHeader tHeader;
		//std::getline(headerFile, temp); cout << line++ << ". " << temp << endl;
		headerFile >> tHeader.timeStamp;
		headerFile >> tHeader.filePath;
		if (tHeader.filePath.size() > 5) dHeader.push_back(tHeader);
	}
	//for_each (ImgHeader var in dHeader) cout << var.timeStamp << " " << var.filePath << endl;

	return 0;
}

int readSensorHeader(std::string fileName, std::vector<SensorHeader> & sHeader)
{
	std::ifstream headerFile;
	headerFile.open(fileName.c_str());

	if (!headerFile.is_open()){ std::cout << "Could not open file\n"; return -1; }

	std::string temp;
	std::getline(headerFile, temp); std::cout << temp << std::endl;
	std::getline(headerFile, temp); std::cout << temp << std::endl;
	std::getline(headerFile, temp); std::cout << temp << std::endl;

	int line = 1;
	while (!headerFile.eof())
	{
		SensorHeader tHeader;
		//std::getline(headerFile, temp); cout << line++ << ". " << temp << endl;
		headerFile >> tHeader.timeStamp;
		headerFile >> tHeader.accl[0];
		headerFile >> tHeader.accl[1];
		headerFile >> tHeader.accl[2];

		if (line == 1){
			sHeader.push_back(tHeader);
		}
		else if (tHeader.timeStamp != sHeader.back().timeStamp){
			sHeader.push_back(tHeader);
		}
		line++;
	}
	return 0;
}

int readGroundTruth(std::string fileName, std::vector<PoseParams> & poseHeaders)
{
	std::ifstream headerFile;
	headerFile.open(fileName.c_str());

	if (!headerFile.is_open()){ std::cout << "Could not open file\n"; return -1; }

	std::string temp;
	std::getline(headerFile, temp); std::cout << temp << std::endl;
	std::getline(headerFile, temp); std::cout << temp << std::endl;
	std::getline(headerFile, temp); std::cout << temp << std::endl;

	int line = 1;
	while (!headerFile.eof())
	{
		PoseParams ps;
		//std::getline(headerFile, temp); cout << line++ << ". " << temp << endl;
		headerFile >> ps.timeStamp;
		headerFile >> ps.tx; headerFile >> ps.ty; headerFile >> ps.tz;
		headerFile >> ps.qx; headerFile >> ps.qy; headerFile >> ps.qz; headerFile >> ps.qw;

		if (line == 1){
			poseHeaders.push_back(ps);
		}
		else if (ps.timeStamp != poseHeaders.back().timeStamp){
			poseHeaders.push_back(ps);
		}
		line++;
	}
	return 0;
}

int readSyncFileHeader(std::string fileName, std::vector<FullFrame> & frame)
{
	std::ifstream headerFile;	headerFile.open(fileName.c_str());
	if (!headerFile.is_open()){ std::cout << "Could not open file\n"; return -1; }
	std::string temp;

	int line = 1;
	std::getline(headerFile, temp); std::cout << temp << std::endl;
	std::getline(headerFile, temp); std::cout << temp << std::endl;
	std::getline(headerFile, temp); std::cout << temp << std::endl;

	std::cout << std::fixed;
	while (!headerFile.eof())	{
		FullFrame tHeader;
		headerFile >> tHeader.depthHeader.timeStamp;
		headerFile >> tHeader.depthHeader.filePath;
		headerFile >> tHeader.rgbHeaders.timeStamp;
		headerFile >> tHeader.rgbHeaders.filePath;

		std::cout << "-Depth: " << tHeader.depthHeader.filePath 
			<< " Color: " << tHeader.rgbHeaders.filePath 
			<< " Time: " << tHeader.depthHeader.timeStamp << std::endl;
		if (tHeader.depthHeader.filePath.size() > 5) frame.push_back(tHeader);
	}
	return 0;
}

int convertToPointcloud(cv::Mat & rgbImg, cv::Mat & depthImg, Intrinsics_TUM cam, PointCloudXYZRGB & _cloud)
{
	_cloud.points.resize(rgbImg.rows*rgbImg.cols);
	_cloud.colors.resize(rgbImg.rows*rgbImg.cols);

	cv::Mat depthImgF;
	depthImg.convertTo(depthImgF, CV_32F); // convert the image data to float type 
	depthImgF /= cam.pixelToMeter;

	const float qnan = std::numeric_limits<float>::quiet_NaN();

	int idx = 0;
	for (int i = 0; i < depthImg.rows; i++)
	{
		for (int j = 0; j < depthImg.cols; j++)
		{
			float d = depthImgF.at<float>(i, j);
			_cloud.colors[idx] = rgbImg.at<cv::Vec3b>(i, j);

			if (d == 0)
				_cloud.points[idx] = cv::Point3f(qnan, qnan, qnan);
			else
				_cloud.points[idx] = cv::Point3f((j - cam.cx) * d / cam.fx, (i - cam.cy) * d / cam.fy, d);
			//_cloud.points[idx] = cv::Point3f((j - cam.cx) * d / cam.fx, -(i - cam.cy) * d / cam.fy, -d);
			idx++;
		}
	}

	return 0;
}


int readPoseFileICL(std::string fileName, std::vector<PoseParams> & frame)
{
	std::ifstream headerFile;	headerFile.open(fileName.c_str());
	
	if (!headerFile.is_open()){ std::cout << "Could not open file\n"; return -1; }
	std::string temp;
	int line = 1;
	std::cout << std::fixed;
	while (!headerFile.eof()){
		double _time;
		PoseParams ps;
		headerFile >> _time;
		headerFile >> ps.tx; headerFile >> ps.ty; headerFile >> ps.tz;
		headerFile >> ps.qx; headerFile >> ps.qy; headerFile >> ps.qz; headerFile >> ps.qw;
		frame.push_back(ps);
	}

	return 0;
}

Eigen::Matrix4d getPose_ICL(const PoseParams & p) //OpenGL convention
{
	Eigen::Matrix3d mat3 = Eigen::Quaterniond(p.qw, p.qx, p.qy, p.qz).toRotationMatrix().transpose();
	Eigen::Matrix4d mat4(Eigen::Affine3d(mat3).matrix());
	mat4(3, 0) = p.tx;
	mat4(3, 1) = p.ty;
	mat4(3, 2) = p.tz;
	return mat4;
}

Eigen::Matrix4d getPose_TUM(const PoseParams & p) //OpenGL convention
{
	Eigen::Matrix3d mat3 = Eigen::Quaterniond(p.qw, p.qx, p.qy, p.qz).toRotationMatrix().transpose();
	Eigen::Matrix4d mat4(Eigen::Affine3d(mat3).matrix());
	mat4(3, 0) = p.tx;
	mat4(3, 1) = p.ty;
	mat4(3, 2) = p.tz;
	return mat4;
}

cv::Affine3d eigen2CV(Eigen::Matrix4d & tMat) //OpenCV convention
{
	double * rawData = tMat.data();
	return cv::Affine3d(rawData);
}
