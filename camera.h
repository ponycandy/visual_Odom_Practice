#ifndef CAMERA_H
#define CAMERA_H

#include "common_include.h"

#include "sl/Camera.hpp"


#include <windows.h>
#include <cuda.h>
#include <cuda_gl_interop.h>


namespace myslam
{
	class Camera
	{
	public:
		Camera(std::string strSettingPath);
		Camera(float fx, float fy, float cx, float cy, float depth_scale) :
			fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale) {}

		/**
		*@brief world2camera ����������ϵ�µ���ά��ת�����������ϵ��
		*@param ���� p_w����������ϵ�µ���ά��
		*@param ���� T_c_w����������ϵ���������ϵ�ı任��ϵ
		*@return  ת�������ά��
		*/
		cv::Point3d world2camera(cv::Point3d p_w, cv::Mat T_c_w);

		/**
		*@brief camera2world ���������ϵ�µ���ά��ת������������ϵ��
		*@param ���� p_c���������ϵ�µ���ά��
		*@param ���� T_w_c���������ϵ����������ϵ�ı任��ϵ
		*@return  ת�������ά��
		*/
		cv::Point3d camera2world(cv::Point3d p_c, cv::Mat T_w_c);

		/**
		*@brief camera2pixel ���������ϵ�µ���ά��ת����ͼ������ϵ��
		*@param ���� p_c���������ϵ�µ���ά��
		*@return  ת�����ͼ������ϵ�µĶ�ά��
		*/
		cv::Point2d camera2pixel(cv::Point3d p_c);

		/**
		*@brief pixel2camera ��ͼ������ϵ�µĶ�ά��ת�����������ϵ��
		*@param ���� p_p��ͼ������ϵ�µĶ�ά��
		*@param ���� depth���ö�ά�����������ϵ�µ����ֵ
		*@return  ת������������ϵ�µ���ά��
		*/
		cv::Point3d pixel2camera(cv::Point2d p_p, double depth = 1.0);

		/**
		*@brief pixel2world ��ͼ������ϵ�µĶ�ά��ת������������ϵ��
		*@param ���� p_p��ͼ������ϵ�µĶ�ά��
		*@param ���� T_w_c���������ϵ����������ϵ�ı任��ϵ
		*@param ���� depth���ö�ά�����������ϵ�µ����ֵ
		*@return  ת�������������ϵ�µ���ά��
		*/
		cv::Point3d pixel2world(cv::Point2d p_p, cv::Mat T_w_c, double depth = 1.0);

		/**
		*@brief world2pixel ����������ϵ�µ���ά��ת����ͼ������ϵ��
		*@param ���� p_w����������ϵ�µ���ά��
		*@param ���� T_c_w����������ϵ���������ϵ�ı任��ϵ
		*@return  ת�����ͼ������ϵ�µĶ�ά��
		*/
		cv::Point2d world2pixel(cv::Point3d p_w, cv::Mat T_c_w);

		sl::Camera* Getcamera();

	public:
		//����ڲ�����
		float fx_, fy_, cx_, cy_;

		//�����ͼת��Ϊʵ�������Ϣ�ĳ߶�ֵ
		float depth_scale_;

		//�������������ȡͼ���buff:
		sl::Mat matGPU_;
	
		//��������󴴽���
		sl::Camera* zed;

		//����������ò�����
		sl::Resolution res;
	};
}
#endif
