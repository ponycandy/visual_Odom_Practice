

#include <iostream>
#include <algorithm>
#include <fstream>


#include <opencv2/core/core.hpp>

#include "system.h"


myslam::Viewer* mpViewer;
std::thread* mptViewer;
cv::Mat im, imD,imDview;
sl::Mat image(1280, 720, sl::MAT_TYPE::U8_C4);
sl::Mat depth_map(1280, 720, sl::MAT_TYPE::U8_C4);
sl::Mat visual_depth_map(1280, 720, sl::MAT_TYPE::U8_C4);

inline int getOCVtype(sl::MAT_TYPE type)
{
	int cv_type = -1;
	switch (type) {
	case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
	default: break;
	}
	return cv_type;
}

inline cv::Mat slMat2cvMat(sl::Mat& input)
{
	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(),
		input.getWidth(), getOCVtype(input.getDataType()),
		input.getPtr<sl::uchar1>(sl::MEM::CPU),
		input.getStepBytes(sl::MEM::CPU));
}

double captureImage(cv::Mat& Img, cv::Mat& ImgD,sl::Camera& zed)
{
	//get tframe here
	double tframe = 0;
	//需要将sl::mat图像转化为cv::mat图像

	
	

	if (zed.grab() == sl::ERROR_CODE::SUCCESS)
	{
		// A new image and depth is available if grab() returns SUCCESS
		zed.retrieveImage(image, sl::VIEW::LEFT); // Retrieve left image
		//zed.retrieveImage(depth_map, sl::VIEW::DEPTH); // Retrieve depth
		zed.retrieveMeasure(depth_map, sl::MEASURE::DEPTH);
		//可能VIEW才是正确的输入，试一试用VIEW的地图能不能转化
		//zed.retrieveImage(visual_depth_map, sl::VIEW::DEPTH);

		sl::Timestamp last_image_timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE);
		tframe = last_image_timestamp.getMicroseconds();
	}

	return tframe;
}
void processDepth(cv::Mat& Image, cv::Mat& ImageDepth, 
	cv::Mat& left, cv::Mat& right, sl::Mat& pointcloud, sl::Camera& cam)
{
	
}
void LoadImages(const std::string& strFile, std::vector<std::string>& vstrImageFilenames, std::vector<double>& vTimestamps)
{
	std::ifstream f;
	f.open(strFile.c_str());

	// skip first three lines
	std::string s0;
	getline(f, s0);
	getline(f, s0);
	getline(f, s0);

	int i = 0;

	while (!f.eof())
	{
		std::string s;
		getline(f, s);
		if (!s.empty())
		{
			std::stringstream ss;
			ss << s;
			double t;
			std::string sRGB;
			ss >> t;
			vTimestamps.push_back(t);
			ss >> sRGB;
			vstrImageFilenames.push_back(sRGB);
		}
	}
}

int main()
{

	std::string strSettingPath = "TUM.yaml";

	myslam::System system(strSettingPath);
	myslam::Camera* cam = system.Get_Camera();

	im = slMat2cvMat(image);
	imD = slMat2cvMat(visual_depth_map);

	cv::Mat leftImage, rightImage;
	sl::Mat Pointcloud;
	while (true)
	{
		double timestamp=captureImage(im, imD , *cam->Getcamera());
		//ushort d=imD.ptr<ushort>(100)[100];
		//std::cout <<"false d : "<< d<<"\n";

		//depth_map
		
		
	/*	float d_actual;
		depth_map.getValue(100, 100, &d_actual);
		std::cout << "real d : " << d_actual << "\n";*/
		cv::Mat T_c_w = system.TrackingRGBD(im, imD, depth_map, timestamp);//这里Im应该没问题，在Qt里面就是这么用的
		
		//相信这里有一定的稳定性，减小搜索空间
	//	cv::imshow("Image", im);
		//cv::imshow("Image", imD);//空的....,所以问题在这里，
		//图像类的东西就是这里麻烦，首先得搞定图像的数据类型
		//看来问题在imD上面
		
		cv::waitKey(10);
	}
}
