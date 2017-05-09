#include "ImgProcCuda.h"
#include "cuda.h"
#include "cuda_runtime.h"
#include "device_types.h"
#include "device_functions.h"
#include "device_launch_parameters.h"

#include "math_functions.h"
#include "math_constants.h"

#define __cds_device__ __device__ __forceinline__

int divUp(int a, int b){ return (a + b - 1) / b; }

__cds_device__ float dot(const float3& v1, const float3& v2)
{
	return __fmaf_rn(v1.x, v2.x, __fmaf_rn(v1.y, v2.y, v1.z*v2.z));
}

__cds_device__ float3 cross(const float3& v1, const float3& v2)
{
	return make_float3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

__cds_device__ float3 operator+(const float3& v1, const float3& v2)
{
	return make_float3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

__cds_device__ float3 operator-(const float3& v1, const float3& v2)
{
	return make_float3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

// multiply
__cds_device__ float3 operator*(const float3& v1, const float3& v2)
{
	return make_float3(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
}

__cds_device__ float3 operator*(float3 a, float s)
{
	return make_float3(a.x * s, a.y * s, a.z * s);
}

__cds_device__ float3 operator*(float s, float3 a)
{
	return make_float3(a.x * s, a.y * s, a.z * s);
}

// normalize
__cds_device__ float3 normalize(const float3& v)
{
	return  rsqrt(dot(v, v)) * v;
}



__global__ void kernel_convert_Depth_To_Point3f(cv::gpu::PtrStep<ushort> _depth, const ImgProc3D::Intr cam, cv::gpu::PtrStep<float3> _point3f)
{
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	float d = float(_depth(y,x)) / cam.scale;
	if (d == 0)
		_point3f(y, x) = { CUDART_NAN_F, CUDART_NAN_F, CUDART_NAN_F };
	else
		_point3f(y, x) = { (x - cam.cx) * d / cam.fx, (y - cam.cy) * d / cam.fy, d };
	return;
}

__global__ void kernel_convert_XYZ_To_Normals(int rows, int cols, cv::gpu::PtrStep<float3> vmap, cv::gpu::PtrStep<float3> nmap)
{
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	if (x >= cols || y >= rows)
		return;

	if (x == cols - 1 || y == rows - 1)
	{
		nmap(y, x) = { CUDART_NAN_F, CUDART_NAN_F, CUDART_NAN_F }; /*CUDART_NAN_F*/
		return;
	}
	float3 v00, v01, v10;

	v00 = vmap(y ,x);
	v01 = vmap(y, x+1);
	v10 = vmap(y+1, x);

	if (!isnan(v00.x) && !isnan(v01.x) && !isnan(v10.x))
	{
		nmap(y, x) = normalize(cross(v01 - v00, v10 - v00));
	}
	else
	{
		//nmap(y, x) = { CUDART_NAN_F, CUDART_NAN_F, CUDART_NAN_F };
		nmap(y, x).x = CUDART_NAN_F;
		nmap(y, x).y = CUDART_NAN_F;
		nmap(y, x).z = CUDART_NAN_F;
	}
}

__global__ void kernel_GenGridMap2D(cv::gpu::PtrStep<float3> _point3f, cv::gpu::PtrStep<uchar3> _rgb,
	float4 pModel,
	float3 pOrg, float3 e_1, float3 e_2,
	cv::gpu::PtrStep<uchar3> _map,
	cv::gpu::PtrStep<uchar> _objMask)
{
		int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	float3 p = _point3f(y, x);

	float3 _pModel = { pModel.x, pModel.y, pModel.z };
	float3 p_new = p - pOrg;
	float p_x_new = dot(e_1, p_new);
	float p_y_new = dot(e_2, p_new);

	int new_x = int(LANE_MAP_SIZE / 2 + p_x_new * LANE_MAP_SCALE);
	int new_y = int(LANE_MAP_SIZE - p_y_new * LANE_MAP_SCALE);

	if (fabs(pModel.x*p.x + pModel.y*p.y + pModel.z*p.z + pModel.w) < PLANE_TEST_THRESHOLD)
	{
		if (new_x > 0 && new_x < LANE_MAP_SIZE  &&  new_y > 0 && new_y < LANE_MAP_SIZE)
		{
			_map(new_y, new_x) = _rgb(y, x);
		}
	}
	else
	{
		float disOne = dot(p, p);
		float disTwo = dot(_pModel, _pModel);

		float ang = dot(p, _pModel) / sqrt(disOne * disTwo);

		if (!isnan(p.z) && p.z < OBJ_DISTANCE_THRESHOLD 
			&& abs(ang) < 0.3f ) {
			_objMask(y, x) = 255;
		}
		
		// if (new_x > 0 && new_x < LANE_MAP_SIZE  &&  new_y > 0 && new_y < LANE_MAP_SIZE)
		// {
		// 	_map(new_y, new_x).x = 0;
		// 	_map(new_y, new_x).y = 0;
		// 	_map(new_y, new_x).z = 255;
		// }
	}

	return;

}

void ImgProc3D::convertTo_Point3fMap(cv::gpu::GpuMat & depth, const ImgProc3D::Intr camInfo, cv::gpu::GpuMat & xyzMat)
{
	cv::Size sz = depth.size();
	dim3 block(32, 16);
	dim3 grid(divUp(sz.width, block.x), divUp(sz.height, block.y));
	kernel_convert_Depth_To_Point3f << <grid, block >> >(depth, camInfo, xyzMat);
}

void ImgProc3D::convertTo_NormalsMap(cv::gpu::GpuMat & xyzMat, cv::gpu::GpuMat & normalMap)
{
	cv::Size sz = xyzMat.size();
	dim3 block(32, 16);
	dim3 grid(divUp(sz.width, block.x), divUp(sz.height, block.y));
	kernel_convert_XYZ_To_Normals << <grid, block >> >(sz.height, sz.width, xyzMat, normalMap);
}

void ImgProc3D::genPlane2DMap(cv::gpu::GpuMat & xyzMat, cv::gpu::GpuMat & rgbMat, cv::Vec4f planeModel, cv::gpu::GpuMat & laneMap, cv::gpu::GpuMat & objMask)
{
	cv::Point3f planeNormal(planeModel[0], planeModel[1], planeModel[2]);
	cv::Point3f e_1 = planeNormal.cross(cv::Point3f(0, 0, 1));
	cv::Point3f e_2 = -planeNormal.cross(e_1);
	cv::Point3f planeOrg = -planeModel[3] * planeNormal;

	cv::Size sz = xyzMat.size();
	dim3 block(32, 16);
	dim3 grid(divUp(sz.width, block.x), divUp(sz.height, block.y));

	float4 _model = { planeModel[0], planeModel[1], planeModel[2], planeModel[3] };
	float3 _pOrg = { planeOrg.x, planeOrg.y, planeOrg.z };
	float3 _e_1 = { e_1.x, e_1.y, e_1.z };
	float3 _e_2 = { e_2.x, e_2.y, e_2.z };
	kernel_GenGridMap2D << <grid, block >> >(xyzMat, rgbMat, _model, _pOrg, _e_1, _e_2, laneMap, objMask);
}