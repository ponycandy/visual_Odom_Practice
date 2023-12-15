#ifndef MAPPOINTS_H
#define MAPPOINTS_H

#include "Frame.h"

namespace myslam
{
	class Frame;
	class MapPoints
	{
	public:
		MapPoints();
		MapPoints(long id, cv::Point3d position, cv::Point3d norm);

	public:
		unsigned long id_;

		cv::Point3d pos_;    //λ��
		cv::Point3d norm_;   //����

		cv::Mat descriptor;  //������
		int observed_times_; //�۲����
		int correct_times_;  //У������

	public:
		MapPoints* creatMappoints();
	};
}
#endif
