#ifndef __IMG_PROC_CUDA_H
#define __IMG_PROC_CUDA_H

//#include <opencv2/core.hpp>
#include <opencv2/core/cuda_types.hpp>
#include <opencv2/core/cuda.hpp>
#include "../types.h"

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

//typedef unsigned short ushort;

//2D Map 512x512 - 1 pixels = 5mm
#define LANE_MAP_SIZE 512
#define LANE_MAP_SCALE 200

void caller_convertTo_Point3fMap(cv::cuda::PtrStep<unsigned short> _depth,
	const ImgProc3D::Intr cam, 
	cv::cuda::PtrStep<float3> _point3f, 
	int width, int height);
void caller_GenGridMap2D(cv::cuda::PtrStep<float3> _point3f, 
	cv::cuda::PtrStep<uchar3> _rgb,
	float4 pModel,
	float3 pOrg, float3 e_1, float3 e_2,
	cv::cuda::PtrStep<uchar3> _map, 
	int width, int height);
#endif // !IMG_PROC_CUDA_H