#ifndef __IMG_PROC_TYPE_H
#define __IMG_PROC_TYPE_H

 //#include <opencv2/core/core.hpp>
// #include <opencv2/core/affine.hpp>
// #include <opencv2/viz/vizcore.hpp>

namespace ImgProc3D
{
    // typedef cv::Matx33f Mat3f;
    // typedef cv::Vec3f Vec3f;
    // typedef cv::Vec3i Vec3i;
    // typedef cv::Affine3f Affine3f;

	enum IntrMode
	{
		IntrMode_640x480_RAW,
		IntrMode_320x240_RAW,
		IntrMode_640x480_IMG,
		IntrMode_320x240_IMG
	};

    struct Intr
    {
		float fx = 570.342165925f;
		float fy = 570.341946943f;
		float cx = 319.5f;
		float cy = 239.5f;
		float scale = 1000.f;

		Intr(float _scale){ scale = _scale; };
		Intr(float _fx, float _fy, float _cx, float _cy, float _scale){
			fx = _fx;	fy = _fy;
			cx = _cx;	cy = _cy;
			scale = _scale;
		};

		Intr(IntrMode mode)
		{
			switch (mode)
			{
			case ImgProc3D::IntrMode_640x480_RAW:
				fx = 570.342165925f;
				fy = 570.341946943f;
				cx = 319.5f;	cy = 239.5f;
				scale = 1000.f;
				break;

			case ImgProc3D::IntrMode_320x240_RAW:
				fx = 285.1710829625f;
				fy = 285.1709734715f;
				cx = 159.5f;	cy = 119.5f;
				scale = 1000.f;

				break;

			case ImgProc3D::IntrMode_640x480_IMG:
				fx = 570.342165925f;
				fy = 570.341946943f;
				cx = 319.5f;	cy = 239.5f;
				scale = 5000.f;
				break;

			case ImgProc3D::IntrMode_320x240_IMG:
				fx = 285.1710829625f;
				fy = 285.1709734715f;
				cx = 159.5f;	cy = 119.5f;
				scale = 5000.f;

				break;
			default:
				break;
			}
		};
        //Intr operator()(int level_index) const;
    };

	/*struct RGBD_Frame
	{
		cv::Mat depthImg;
		cv::Mat rgbImg;
	};*/

    std::ostream& operator << (std::ostream& os, const Intr& intr);

    struct Point
    {
        union
        {
            float data[4];
            struct { float x, y, z; };
        };
    };

    typedef Point Normal;

    struct RGB
    {
        union
        {
            struct { unsigned char b, g, r; };
            int bgra;
        };
    };

    struct PixelRGB
    {
        unsigned char r, g, b;
    };

    inline float deg2rad (float alpha) { return alpha * 0.017453293f; }
}

#endif