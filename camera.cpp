#include "camera.h"
namespace myslam
{
	Camera::Camera(std::string strSettingPath)
	{
		//cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

		//fx_ = fSettings["Camera.fx"];
		//fy_ = fSettings["Camera.fy"];
		//cx_ = fSettings["Camera.cx"];
		//cy_ = fSettings["Camera.cy"];
		//depth_scale_ = fSettings["DepthMapFactor"];

		//初始化摄像机：


		zed = new sl::Camera;

		// Open the camera
		sl::ERROR_CODE returned_state = zed->open();
		if (returned_state != sl::ERROR_CODE::SUCCESS)
		{
			std::cout << "Error " << returned_state << ", exit program.\n";
			return;
		}

		// Get camera information (ZED serial number)
		auto camera_config = zed->getCameraInformation().camera_configuration;
		//获取分辨率,焦距，相机初始点，depth单位（scale）等等
		fx_=camera_config.calibration_parameters.left_cam.fx;
		fy_ = camera_config.calibration_parameters.left_cam.fy;
		cx_= camera_config.calibration_parameters.left_cam.cx;
		cy_ = camera_config.calibration_parameters.left_cam.cy;
		depth_scale_ = 1000;//因为深度适用mm表示，需要转化为m表示，所以除以1000
	}

	cv::Point3d Camera::world2camera(cv::Point3d p_w, cv::Mat T_c_w)
	{
		cv::Mat pts3_w(p_w);
		cv::Mat pts4_w(4, 1, CV_64FC1);
		pts3_w.copyTo(pts4_w(cv::Rect(0, 0, 1, 3)));
		pts4_w.at<double>(3, 0) = 1.0;

		cv::Mat pts_convert = T_c_w * pts4_w;
		return cv::Point3d(pts_convert(cv::Rect(0, 0, 1, 3)));
	}

	cv::Point3d Camera::camera2world(cv::Point3d p_c, cv::Mat T_w_c)
	{
		cv::Mat pts3_c(p_c);
		cv::Mat pts4_c(4, 1, CV_64FC1);
		pts3_c.copyTo(pts4_c(cv::Rect(0, 0, 1, 3)));
		pts4_c.at<double>(3, 0) = 1.0;

		cv::Mat pts_convert = T_w_c * pts4_c;
		return cv::Point3d(pts_convert(cv::Rect(0, 0, 1, 3)));
	}

	cv::Point2d Camera::camera2pixel(cv::Point3d p_c)
	{
		cv::Mat pts3_c(p_c);
		return cv::Point2d(
			fx_ * pts3_c.at<double>(0, 0) / pts3_c.at<double>(2, 0) + cx_,
			fy_ * pts3_c.at<double>(1, 0) / pts3_c.at<double>(2, 0) + cy_
		);
	}

	cv::Point3d Camera::pixel2camera(cv::Point2d p_p, double depth)
	{
		//立体视觉也是这个办法，关键在于depth
		cv::Mat pts_p(p_p);
		return cv::Point3d(
			(pts_p.at<double>(0, 0) - cx_) * depth / fx_,
			(pts_p.at<double>(1, 0) - cy_) * depth / fy_,
			depth
		);
		//ZED唤起！
	}

	cv::Point2d Camera::world2pixel(cv::Point3d p_w, cv::Mat T_c_w)
	{
		return camera2pixel(world2camera(p_w, T_c_w));
	}

	sl::Camera* Camera::Getcamera()
	{
		return zed;
	}

	cv::Point3d Camera::pixel2world(cv::Point2d p_p, cv::Mat T_w_c, double depth)
	{
		return camera2world(pixel2camera(p_p, depth), T_w_c);
	}
}

