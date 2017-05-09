#include "LaneDetector.h"
#include <fstream>

// #define DEBUG
#define USE_CUDA_LANE_DETECTION
#define USE_STATIC_PLANE_MODEL


#ifdef DEBUG
    #define debug(x) cerr << #x << ": " << x << std::endl;
#else
    #define debug(x)
#endif

static float qnan = std::numeric_limits<float>::quiet_NaN();

cv::Vec4f LoadCalibFile() {
    FILE *inp = fopen("calib.txt", "r");
    cv::Vec3f p;
    cv::Vec3f in;
    int n = 100;
    float d=0, din=0;

    p = cv::Vec3f(0, 0, 0); 
    for (int i = 0; i < n; i++) {
        fscanf(inp, "%f%f%f%f", &in[0], &in[1], &in[2], &din);
        p += in;
        d += din;
    }
    fclose(inp);

    cv::Vec3f v1 = cv::normalize(p);

    d /= n;    
    return cv::Vec4f(v1[0], v1[1], v1[2], d);
}

ImgProc3D::LaneDetector::LaneDetector(int imgWidth, int imgHeight) :m_camInfo(5000.f)
{
	smallImage = false;
	xyzMap = cv::Mat(imgHeight, imgWidth, CV_32FC3);
	laneProject2D = cv::Mat(imgHeight, imgWidth, CV_32FC2, cv::Scalar(qnan, qnan));
	laneMap2D = cv::Mat(LANE_MAP_SIZE, LANE_MAP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0));
	objectMask = cv::Mat(imgHeight, imgWidth, CV_8UC1, cv::Scalar(0));
    iniLeft = cv::Point(0, 0); 
	iniRight = cv::Point(0, 0);
	
#ifdef USE_CUDA_LANE_DETECTION
	dev_dMat = cv::gpu::GpuMat(imgHeight, imgWidth,CV_16UC1);
	dev_rgbMat = cv::gpu::GpuMat(imgHeight, imgWidth, CV_8UC3);
	dev_xyzMap = cv::gpu::GpuMat(imgHeight, imgWidth, CV_32FC3);
	dev_laneMap2D = cv::gpu::GpuMat(LANE_MAP_SIZE, LANE_MAP_SIZE, CV_8UC3, cv::Scalar(0, 0, 0));
	dev_objectMask = cv::gpu::GpuMat(imgHeight, imgWidth, CV_8UC1, cv::Scalar(0));
#endif
    
#ifdef USE_STATIC_PLANE_MODEL
	// Init Plane planeModel
	planeModel = LoadCalibFile();
	printf("Load plane model %f	%f %f %f \n", planeModel[0], planeModel[1], planeModel[2], planeModel[3]);
#endif // USE_STATIC_PLANE_MODEL
}

ImgProc3D::LaneDetector::~LaneDetector ()
{
	
}

ImgProc3D::Intr ImgProc3D::LaneDetector::getCamInfor() {
    return m_camInfo;
}

void ImgProc3D::LaneDetector::setCamera(ImgProc3D::IntrMode _mode)
{
	m_camInfo = ImgProc3D::Intr(_mode);
	if (_mode == ImgProc3D::IntrMode_320x240_IMG || _mode == ImgProc3D::IntrMode_320x240_RAW) {	smallImage = true; }
	else {	smallImage = false;	}
}

void ImgProc3D::LaneDetector::calibFrame(cv::Mat & rgbMat, cv::Mat & dMat)
{
	std::ofstream ofs;
	ofs.open("calib.txt", std::ofstream::out | std::ofstream::app);

	cv::Vec4f t_planeModel = fast_PlaneDetect(dMat, rgbMat, m_camInfo, smallImage);
	if (planeModel[1] != 1.0f)	{
		ofs << t_planeModel[0] << " " << t_planeModel[1] << " " << t_planeModel[2] << " " << t_planeModel[3] << "\n";
	}
	else{
		std::cout << "WARNING: CANNOT FIND ANY PLANE ... \n";
	}

	ofs.close();
}

void ImgProc3D::LaneDetector::processFrame(cv::Mat & rgbMat, cv::Mat & dMat)
{
#ifdef USE_STATIC_PLANE_MODEL
	//DO NOTHING
#else
	planeModel = fast_PlaneDetect(dMat, rgbMat, m_camInfo, smallImage);
	if (planeModel[1] == 1.0f)	{
		planeModel = m_previous_plane_model;
	}	else	{
		m_previous_plane_model = planeModel;
	}
#endif // USE_STATIC_PLANE_MODEL

#ifdef USE_CUDA_LANE_DETECTION
	dev_dMat.upload(dMat);
	dev_rgbMat.upload(rgbMat);
	
	ImgProc3D::convertTo_Point3fMap(dev_dMat, m_camInfo, dev_xyzMap);

	dev_laneMap2D.setTo(cv::Scalar(128, 128, 128));
	dev_objectMask.setTo(cv::Scalar(0));
	ImgProc3D::genPlane2DMap(dev_xyzMap, dev_rgbMat, planeModel, dev_laneMap2D, dev_objectMask);
	dev_laneMap2D.download(laneMap2D);
	dev_objectMask.download(objectMask);

#else
	convertToXYZ(dMat, m_camInfo, xyzMap);

	laneProject2D = cv::Scalar(qnan, qnan);
	fillLaneMap2D(xyzMap, laneProject2D, planeModel);

	laneMap2D = cv::Scalar(100, 100, 100);
	create2DGrid(laneProject2D, rgbMat, laneMap2D);
#endif // USE_CUDA_LANE_DETECTION
}

void ImgProc3D::LaneDetector::copyPoint(cv::Point src, cv::Point& dst) {
    dst.x = src.x;  dst.y = src.y;
}

std::vector<cv::Point> ImgProc3D::LaneDetector::seederPostProcessing(std::vector<cv::Point> seeder) {
    cv::Point3f equa;
    float distance, thresh = MaskSize;
    int cnt;
    std::vector<cv::Point> output;

    if (seeder.size() < 3)
        return seeder;

    for (int i = 0; i < seeder.size(); i++)
        for (int j = i + 1; j < seeder.size(); j++) {
            line_equation(seeder[i], seeder[j], equa);
            cnt = 0;
            for (int k = 0; k < seeder.size(); k++) {
                distance = abs(equa.x * seeder[k].x + equa.y * seeder[k].y + equa.z);
                distance /= std::sqrt(sqr(equa.x) + sqr(equa.y));
                if (distance < thresh)
                    cnt++;
            }

            if (cnt > seeder.size() * 7 / 10) {
                for (int k = 0; k < seeder.size(); k++) {
                    distance = abs(equa.x * seeder[k].x + equa.y * seeder[k].y + equa.z);
                    distance /= std::sqrt(sqr(equa.x) + sqr(equa.y));
                    if (distance < thresh)
                        output.push_back(seeder[k]);
                }

                return output;
            }
        }
    return seeder;
}

void ImgProc3D::LaneDetector::findInitialPoint(const cv::Mat& image, cv::Point& pLeft, cv::Point& pRight,
     		bool& flagLeft, bool& flagRight, int offsetX, int offsetY) 
{
    int cnt, best_cnt_left = 0, best_cnt_right = 0;
	int rows = image.rows, cols = image.cols;
    float thre_white = MaskSize * MaskSize * 6 / 10;

    cv::Point point(0, image.rows);
    pLeft = cv::Point(point.x, point.y);
    pRight = cv::Point(point.x, point.y);

    iniLefts.clear();
    iniRts.clear();

    while (point.y > rows * 2 / 3) {
        point.y -= offsetY;
        point.x = 0;

        best_cnt_left = 0; best_cnt_right = 0;

        while (point.x < cols) { // ACCESS ERRORS !!!
            point.x = point.x + offsetX;
			if (point.x < 0 || point.x > cols)
                continue;
            cnt = count_white(image, point, MaskSize);
            if (point.x <= cols / 2) {
                if (cnt > thre_white && cnt > best_cnt_left) {
                    best_cnt_left = cnt;
                    copyPoint(point, pLeft);
                }
            } else {
                if (cnt > thre_white && cnt > best_cnt_right) {
                    best_cnt_right = cnt;
                    copyPoint(point, pRight);
                }
            }
        }
        
        if (best_cnt_left > 0) 
            iniLefts.push_back(pLeft);
        
        if (best_cnt_right > 0)
            iniRts.push_back(pRight);
    }

    if (iniLefts.size() > 0) {
        flagLeft = true;
        iniLefts = seederPostProcessing(iniLefts);
        copyPoint(iniLefts[0], pLeft);
    }

    if (iniRts.size() > 0) {
        flagRight = true;
        iniRts = seederPostProcessing(iniRts);
        copyPoint(iniRts[0], pRight);
    }
}
        
void ImgProc3D::LaneDetector::lineProcessing(const cv::Mat& image, cv::Point first, int flag, std::vector<cv::Point>& line, 
    int offsetX, int offsetY) 
{
    cv::Point point(first.x, first.y);
    cv::Point best_p(point.x, point.y);
    cv::Point real_p(first.x, first.y);
    int cnt, best_cnt = 0, diff = 0, rows = image.rows, cols = image.cols;
    float thre_lane = MaskSize * MaskSize * 2 / 10;

    line.push_back(first);

    while (true) {
        point.y -= offsetY;
        if (point.y < 0)
            break;
        best_cnt = 0;

        int prev_x = point.x;
        for (int k = -3; k <= 3; k++) {
            point.x = prev_x + k * offsetX;
            if (point.x < 0 || point.x > cols)
                continue;
            cnt = count_white(image, point, 9);
            if (cnt > thre_lane && cnt > best_cnt) {
                best_cnt = cnt;
                copyPoint(point, best_p);
            }
        }
        
        if (best_cnt > 0) {
            diff = (best_p.x - real_p.x) / 3;
            copyPoint(best_p, point);
            copyPoint(best_p, real_p);
            line.push_back(point);
        } else {
            point.x = prev_x + diff;
        }
    }
}

void ImgProc3D::LaneDetector::findLane(const cv::Mat& image) 
{
    cv::Point pLeft(0, 0);
    cv::Point pRight(0, 0);
    bool flagLeft = false, flagRight = false;

    iniLeft = cv::Point(-1, -1);
    iniRight = cv::Point(-1, -1);

    findInitialPoint(image, iniLeft, iniRight, flagLeft, flagRight, 3, 11);
    
    left_line.clear();
    right_line.clear();
    cv::Point3f line_eq;
    if (flagLeft) {  
        lineProcessing(image, iniLeft, 0, left_line, 3, 11);
        if (!flagRight) {
            int laneSize = 120;
            for (int i = 1; i < left_line.size(); i++) 
            {
                line_equation(left_line[i - 1], left_line[i], line_eq);
                float tmp = sqrt(line_eq.x * line_eq.x + line_eq.y * line_eq.y);
                cv::Point2f p;
                p.y = left_line[i - 1].y - line_eq.y * laneSize / tmp;
                p.x = left_line[i - 1].x + MaskSize - line_eq.x * laneSize / tmp;
                right_line.push_back(p);
            }
        }
    }
    
    if (flagRight) {
        lineProcessing(image, iniRight, 1, right_line, 3, 11);
        if (!flagLeft) {
            int laneSize = 120;
            for (int i = 1; i < right_line.size(); i++) 
            {
                line_equation(right_line[i - 1], right_line[i], line_eq);
                float tmp = sqrt(line_eq.x * line_eq.x + line_eq.y * line_eq.y);
                cv::Point2f p;
                p.y = right_line[i - 1].y + line_eq.y * laneSize / tmp;
                p.x = right_line[i - 1].x + line_eq.x * laneSize / tmp;
                left_line.push_back(p);
            }
        }
    }
}

std::vector<cv::Point2f> ImgProc3D::LaneDetector::findLaneCenter(const cv::Mat& image, 
							int radius, bool useLeftLane, bool useRightLane) 
{
    float a, b;
    cv::Point center(0, 0);
    std::vector<cv::Point2f> centers, centers_cmb;
    cv::Point prev_p;
    cv::Point3f line_eq;
    int widthLane = 120;

        if (left_line.size() > right_line.size()) {
            if (useRightLane == true) radius = widthLane - radius;
        // center from left
            for (int i = 1; i < left_line.size(); i++) 
            {
                line_equation(left_line[i - 1], left_line[i], line_eq);
                float tmp = sqrt(line_eq.x * line_eq.x + line_eq.y * line_eq.y);
                center.y = left_line[i - 1].y - line_eq.y * radius / tmp;
                center.x = left_line[i - 1].x + MaskSize - line_eq.x * radius / tmp;
                centers.push_back(cv::Point2f(center.x, center.y));
            }
        } else {
            if (useLeftLane == true) radius = widthLane - radius;
        // center from right
            for (int i = 1; i < right_line.size(); i++) 
            {
                line_equation(right_line[i - 1], right_line[i], line_eq);
                float tmp = sqrt(line_eq.x * line_eq.x + line_eq.y * line_eq.y);
                center.y = right_line[i - 1].y + line_eq.y * radius / tmp;
                center.x = right_line[i - 1].x + line_eq.x * radius / tmp;
                centers.push_back(cv::Point2f(center.x, center.y));
            }
        }        
    
    // combine center
    // std::sort(centers.begin(), centers.end(), cmp_function);

    // fix bug
    if (centers.size() == 0)
        return centers;

    prev_p = cv::Point(centers[0].x, 1000);
    for (int i = 0; i < centers.size(); i++)  
        if (prev_p.y - centers[i].y >= 15)
    {
        prev_p.x = centers[i].x;
        prev_p.y = centers[i].y;
        centers_cmb.push_back(cv::Point2f(centers[i].x, 512 - centers[i].y));
    }

    return centers_cmb;
}

bool ImgProc3D::LaneDetector::detectLaneCenter(std::vector<cv::Point2f> centers, cv::Point2f& center) {
	
    // std::cout << centers << std::endl;
    float dis = 100000;
    int id = -1;

    // 1 pixel = 5 mm
    // 100 pixel  = 500 mm = 50 cm
    for (int i = 0; i < centers.size(); i++) {
        if (fabs(std::sqrt((sqr(centers[i].x - 256) + sqr(centers[i].y))) - 100) < dis) {
            id = i;
            dis = fabs(std::sqrt((sqr(centers[i].x - 256) + sqr(centers[i].y))) - 100);
        }
    }

    if (id == -1)
        return false;

    center = cv::Point2f(centers[id].x, centers[id].y);
    return true;
}

cv::Point ImgProc3D::LaneDetector::projectToMap2D(const cv::Mat & dMat, cv::Point & left, cv::Point & right){
    cv::Point tmp = getCenterObj(objectMask, left, right);
	if (tmp.x < 0) return cv::Point(-1, -1);
	ImgProc3D::Intr camInfor = getCamInfor();

	cv::Point3f tmpXyz = getPointXYZ(dMat, camInfor,  tmp);
    cv::Point3f tmpLeft = getPointXYZ(dMat, camInfor, left);
    cv::Point3f tmpRight = getPointXYZ(dMat, camInfor, right);
	
    cv::Point3f objXYZ = projectPointToPlane(tmpXyz, planeModel);
    cv::Point3f objLeft = projectPointToPlane(tmpLeft, planeModel);
    cv::Point3f objRight = projectPointToPlane(tmpRight, planeModel);
	
    cv::Point xy = findPointIn2dMap(objXYZ, planeModel, laneMap2D.cols, laneMap2D.rows );
    left = findPointIn2dMap(objLeft, planeModel, laneMap2D.cols, laneMap2D.rows );
    right = findPointIn2dMap(objRight, planeModel, laneMap2D.cols, laneMap2D.rows );
	
    return xy;
}

float ccw(cv::Point2f p1, cv::Point2f p2, cv::Point2f p3) { 
    // p1 -> p2 -> p3
    cv::Point2f vec1 = cv::Point2f(p2.x - p1.x, p2.y - p1.y);
    cv::Point2f vec2 = cv::Point2f(p3.x - p2.x, p3.y - p2.y);

    return vec1.x * vec2.y - vec1.y * vec2.x;
}

int ImgProc3D::LaneDetector::checkObsPosition(std::vector<cv::Point> left, std::vector<cv::Point> right, cv::Point obs) {
    int lsize = left.size();
    int rsize = right.size();
    std::vector<cv::Point2f> realLeft; realLeft.clear();
    std::vector<cv::Point2f> realRight; realRight.clear();
        
    
    cv::Point2f p;
    for (int i = 0; i < lsize; i++) {
        p.x = (left[i].x - 256) * 0.5;
        p.y = left[i].y * 0.5;
        realLeft.push_back(p);
    }
    for (int i = 0; i < rsize; i++) {
        p.x = (right[i].x - 256) * 0.5;
        p.y = right[i].y * 0.5;
        realRight.push_back(p);
    }
    
    cv::Point2f realObj;
    realObj.x = (obs.x - 256.0) * 0.5;
    realObj.y = (512.0 - obs.y) * 0.5;

    double leftDist = projectorDistance(realLeft, realObj);
    double rightDist = projectorDistance(realRight, realObj);
    if (leftDist < rightDist) return -1; else return 1;
}