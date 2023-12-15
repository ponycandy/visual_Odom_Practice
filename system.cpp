#include "system.h"


namespace myslam
{
	System::System(const string& strSettingPath)
	{
		//初始化界面显示线程,显示线程可以首先不管
		mpViewer = new Viewer(strSettingPath);
		mptViewer = new thread(&Viewer::Run, mpViewer);

		//初始化视觉里程计指针
		m_pVO = new myslam::VisualOdometry(strSettingPath, mpViewer);
	}

	cv::Mat System::TrackingRGBD(cv::Mat im, cv::Mat imD, double tframe)
	{
		//开始跟踪
		return m_pVO->Tracking(im, imD, tframe);
	}
	myslam::Camera* System::Get_Camera()
	{
		return m_pVO->mp_camera;
	}
}
