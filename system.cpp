#include "system.h"


namespace myslam
{
	System::System(const string& strSettingPath)
	{
		//��ʼ��������ʾ�߳�,��ʾ�߳̿������Ȳ���
		mpViewer = new Viewer();

		//��ʼ���Ӿ���̼�ָ��
		m_pVO = new myslam::VisualOdometry(strSettingPath, mpViewer);
	}

	cv::Mat System::TrackingRGBD(cv::Mat im, cv::Mat imD, sl::Mat ZedIMD, double tframe)
	{
		//��ʼ����
		return m_pVO->Tracking(im, imD, ZedIMD, tframe);
	}
	myslam::Camera* System::Get_Camera()
	{
		return m_pVO->mp_camera;
	}
}
