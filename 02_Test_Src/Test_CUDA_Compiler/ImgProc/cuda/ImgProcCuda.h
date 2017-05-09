#ifndef __IMG_PROC_CUDA_H
#define __IMG_PROC_CUDA_H

#include <opencv2/core.hpp>

#include "opencv2/core/devmem2d.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "../types.h"

//2D Map 512x512 - 1 pixels = 5mm
#define LANE_MAP_SIZE 512
#define LANE_MAP_SCALE 200
#define OBJ_DISTANCE_THRESHOLD 1.5f

namespace ImgProc3D
{
	void convertTo_Point3fMap(cv::gpu::GpuMat & depthMat, const ImgProc3D::Intr camInfo, cv::gpu::GpuMat & xyzMat);
	void convertTo_NormalsMap(cv::gpu::GpuMat & xyzMat, cv::gpu::GpuMat & normalMap);
	void genPlane2DMap(cv::gpu::GpuMat & xyzMat, cv::gpu::GpuMat & rgbMat, cv::Vec4f planeModel, cv::gpu::GpuMat & laneMap, cv::gpu::GpuMat & objMask);
}

#endif