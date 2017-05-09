#include "ImgProcCuda.h"

int divUp(int a, int b){ return (a + b - 1) / b; }

__device__ float dot(const float3& v1, const float3& v2)
{
	return __fmaf_rn(v1.x, v2.x, __fmaf_rn(v1.y, v2.y, v1.z*v2.z));
}

__device__ float3 cross(const float3& v1, const float3& v2)
{
	return make_float3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

__device__ float3 operator+(const float3& v1, const float3& v2)
{
	return make_float3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

__device__ float3 operator-(const float3& v1, const float3& v2)
{
	return make_float3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

__global__ void kernel_convert_Depth_To_Point3f(cv::cuda::PtrStep<unsigned short> _depth, const ImgProc3D::Intr cam, cv::cuda::PtrStep<float3> _point3f)
{
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	float d = float(_depth(y, x)) / cam.scale;
	_point3f(y, x) = { (x - cam.cx) * d / cam.fx, (y - cam.cy) * d / cam.fy, d };
	return;
}

__global__ void kernel_GenGridMap2D(cv::cuda::PtrStep<float3> _point3f, cv::cuda::PtrStep<uchar3> _rgb,
	float4 pModel,
	float3 pOrg, float3 e_1, float3 e_2,
	cv::cuda::PtrStep<uchar3> _map)
{
	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	float3 p = _point3f(y, x);

	float3 p_new = p - pOrg;
	float p_x_new = dot(e_1, p_new);
	float p_y_new = dot(e_2, p_new);

	int new_x = int(LANE_MAP_SIZE / 2 + p_x_new * LANE_MAP_SCALE);
	int new_y = int(LANE_MAP_SIZE - p_y_new * LANE_MAP_SCALE);

	if (fabs(pModel.x*p.x + pModel.y*p.y + pModel.z*p.z + pModel.w) < 0.05f)
	{
		if (new_x > 0 && new_x < LANE_MAP_SIZE  &&  new_y > 0 && new_y < LANE_MAP_SIZE)
		{
			_map(new_y, new_x) = _rgb(y, x);
		}
	}
	else
	{
		if (new_x > 0 && new_x < LANE_MAP_SIZE  &&  new_y > 0 && new_y < LANE_MAP_SIZE)
		{
			_map(new_y, new_x) = { 0, 0, 255 };
		}
	}

	return;
}

void caller_convertTo_Point3fMap(cv::cuda::PtrStep<unsigned short> _depth, const ImgProc3D::Intr cam, cv::cuda::PtrStep<float3> _point3f, int width, int height)
{
	/*cv::Size sz = depth.size();*/
	dim3 block(32, 16);
	dim3 grid(divUp(width, block.x), divUp(height, block.y));
	kernel_convert_Depth_To_Point3f << <grid, block >> >(_depth, cam, _point3f);
}

void caller_GenGridMap2D(cv::cuda::PtrStep<float3> _point3f, cv::cuda::PtrStep<uchar3> _rgb,
	float4 pModel,
	float3 pOrg, float3 e_1, float3 e_2,
	cv::cuda::PtrStep<uchar3> _map, int width, int height)
{
	dim3 block(32, 16);
	dim3 grid(divUp(width, block.x), divUp(height, block.y));
	kernel_GenGridMap2D << <grid, block >> >(_point3f, _rgb, pModel, pOrg, e_1, e_2, _map);
}