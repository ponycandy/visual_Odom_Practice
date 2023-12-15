#ifndef SYSTEM_H
#define SYSTEM_H

#include <fstream>
#include <string.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <thread>
#include <iomanip>

#include "viewer.h"
#include "visual_odometry.h"
#include "camera.h"

namespace myslam
{
	using namespace std;
	class System
	{
	public:
		System(const string& strSettingPath);

		cv::Mat TrackingRGBD(cv::Mat im, cv::Mat imD,sl::Mat ZedIMD, double tframe);
		myslam::Camera* Get_Camera();
	public:
		Viewer* mpViewer;
		std::thread* mptViewer;

		VisualOdometry* m_pVO;
	};
}

#endif 
