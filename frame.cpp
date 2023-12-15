#include "frame.h"
#include "frame.h"

namespace myslam
{
	Frame::Frame() :id_(-1), time_stamp_(-1), camera_(nullptr) {}
	Frame::Frame(long id, double time_stamp, cv::Mat T_c_w, Camera* camera, cv::Mat color, cv::Mat depth,sl::Mat depthzd) :
		id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth),depth_zed(depthzd)
	{

	}

	double Frame::findDepth(const cv::KeyPoint& kp)
	{
		//这里需要更改为适用于stereo的类型！
		//输入的是像素的坐标！
		int x = round(kp.pt.x);
		int y = round(kp.pt.y);

	//	ushort d = depth_.ptr<ushort>(y)[x];
		float d_actual;
		depth_zed.getValue(x, y, &d_actual);
		if (d_actual != 0)
			return d_actual / camera_->depth_scale_;
		else
		{
			//没有找到合适的深度值时,在其四领域内再次查找
			int dx[4] = { -1,0,1,0 };
			int dy[4] = { 0,-1, 0, 1 };
			for (int i = 0; i < 4; i++)
			{
				depth_zed.getValue(x + dx[i], y + dy[i], &d_actual);
				//d_actual = depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
				if (d_actual != 0)
					return d_actual / camera_->depth_scale_;
			}
		}
		return -1.0;
	}

	cv::Mat Frame::getCamCenter() const
	{
		cv::Mat T_w_c = T_c_w_.inv(cv::DecompTypes::DECOMP_SVD);
		return T_w_c(cv::Rect(3, 0, 1, 3));
	}

	bool Frame::isInFrame(const cv::Point3d pt_world)
	{
		cv::Point3d p_cam = camera_->world2camera(pt_world, T_c_w_);
		if (p_cam.z < 0)
			return false;
		cv::Point2d pixel = camera_->camera2pixel(p_cam);
		return (pixel.x > 0 && pixel.y > 0
			&& pixel.x < color_.cols && pixel.y < color_.rows);
	}
}
