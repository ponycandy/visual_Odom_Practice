#include "visual_odometry.h"

namespace myslam
{
	VisualOdometry::VisualOdometry(std::string strSettingPath, Viewer* pViewer) :
		state_(INITIALZING), ref_(nullptr), curr_(nullptr), map_(new Map), num_lost_(0), num_inliers_(0), mpViewer(pViewer)
	{
		//��ȡ�����ļ�
		//cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
		//�Ҳ������漸������ɶ��˼��
		//num_of_features_ = fSettings["number_of_features"];
		//scale_factor_ = fSettings["scale_factor"];
		//level_pyramid_ = fSettings["level_pyramid"];
		//match_ratio_ = fSettings["match_ratio"];
		//max_num_lost_ = fSettings["max_num_lost"];
		//min_inliers_ = fSettings["min_inliers"];
		//key_frame_min_rot = fSettings["keyframe_rotation"];
		//key_frame_min_trans = fSettings["keyframe_translation"];
		num_of_features_ = 500;
		scale_factor_ = 1.2;
		level_pyramid_ = 8;
		match_ratio_ = 1;//δ��ʹ�õĲ���
		max_num_lost_ = 150;//��ඪ������֮һ�ĵ���Ƕ�ʧ
		min_inliers_ = 150;//��֪�������в���
		key_frame_min_rot =0.1;//��֪������������ȽϺ���
		key_frame_min_trans = 0.1;//��֪������������ȽϺ���
		//fSettings.release();

		//���������
		mp_camera = new myslam::Camera(strSettingPath);

		//����orb������ȡ��
		orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);

		frameid = 0;
	}

	VisualOdometry::~VisualOdometry()
	{

	}

	//����
	cv::Mat VisualOdometry::Tracking(cv::Mat im, cv::Mat imD,sl::Mat ZEDIMD, double tframe)
	{
		//����ͼ��֡
		Frame* pFrame = new Frame(frameid);
		pFrame->color_ = im;
		pFrame->depth_ = imD;
		pFrame->depth_zed = ZEDIMD;
		pFrame->camera_ = mp_camera;
		pFrame->time_stamp_ = tframe;//�ⶫ������û���ã�

		//��ø���λ����Ϣ
		cv::Mat T_c_w = addFrame(pFrame);

		frameid++;

		return T_c_w;
	}

	cv::Mat VisualOdometry::addFrame(Frame* frame)
	{
		cv::Mat T_c_w;

		//δ��ʼ��
		if (state_ == INITIALZING)
		{
			std::unique_lock<std::mutex> lock(mpViewer->mMutexViewer);

			//��ǰ֡,�ο�֡����
			curr_ = ref_ = frame;
			ref_->T_c_w_ = cv::Mat::eye(4, 4, CV_32FC1);
			curr_->T_c_w_ = cv::Mat::eye(4, 4, CV_32FC1);

			//��ǰ֡�����ͼ
			map_->insertKeyFrame(frame);

			//��ȡORB����
			ExtractORB();

			//����������Ӧ����ά��
			setRef3DPoints();

			state_ = OK;
			return curr_->T_c_w_;
		}
		else
		{
			//���ٶ�ʧ
			if (state_ == LOST)
			{
				std::cout << "vo has lost." << std::endl;

				num_lost_++;
				if (num_lost_ > max_num_lost_)
				{
					state_ = LOST;
				}
				return ref_->T_c_w_;
			}
			else
			{
				//λ�˸���
				curr_ = frame;
				ExtractORB();

				//����ƥ��
				featureMatching();
				//if (mathe_flag == 0)
				//{
				//	return cv::Mat();
				//}

				//������һ֡�뵱ǰ֡��λ�ù�ϵ
				poseEstimationPnP();

				//���λ���Ƿ���ȷ
				if (checkEstimatedPose() == true)
				{
					//���㵱ǰ֡�������������ϵ��λ��
					curr_->T_c_w_ = T_c_r_estimate * ref_->T_c_w_;
					ref_ = curr_;

					//���㵱ǰ֡����ά��
					setRef3DPoints();
					num_lost_ = 0;

					//�Ƿ��ǹؼ�֡
					if (checkKeyFrame() == true)
					{
						addKeyFrame();
					}
				}

				//���е�ͼ��֡
				mvAllFrame.push_back(curr_);

				cv::Mat T_c_w = curr_->T_c_w_;

				//����ʾ����������ŵ�����,��ʾ�����ǵ�������������һ����ʾ��viewer�����
				//mpViewer->SetCurrentCameraPose(T_c_w);
				//mpViewer->GetAllFrame(mvAllFrame);
				//mpViewer->GetAll3dPoints(pts_3d_all);
				//mpViewer->SetVisualOdometry(this);

				cv::waitKey(10);

				return T_c_w;
			}
		}
	}

	//��ȡ��������������
	void VisualOdometry::ExtractORB()
	{
		orb_->detectAndCompute(curr_->color_, cv::Mat(), keypoints_curr_, descriptors_curr_);
	}

	//����ƥ�䣬BF+������ֵɸѡ
	void VisualOdometry::featureMatching()
	{
		std::vector<cv::DMatch> matches;
		cv::BFMatcher matcher(cv::NORM_HAMMING);

		matcher.match(descriptors_ref_, descriptors_curr_, matches);
		float max_dis = 0;
		for (int i = 0; i < matches.size(); i++)
		{
			if (matches[i].distance > max_dis)
				max_dis = matches[i].distance;
		}

		feature_matches_.clear();
		for (cv::DMatch& m : matches)
		{
			if (m.distance < max_dis * 0.3)
				feature_matches_.push_back(m);
		}
		std::cout << "good matches: " << feature_matches_.size() << std::endl;
		if (feature_matches_.size() == 0)
		{
			mathe_flag = 0;
		}
	}

	//����ÿ���������Ӧ����ά��
	//���������������Ҫһ�������ĵ������Ի�ȡ��Ӧ���ص����Ȼ�������
	void VisualOdometry::setRef3DPoints()
	{
		pts_3d_ref.clear();
		descriptors_ref_ = cv::Mat();
		for (int i = 0; i < keypoints_curr_.size(); i++)
		{
			double d = ref_->findDepth(keypoints_curr_[i]);

			cv::Point3f p_cam = ref_->camera_->pixel2camera(cv::Point2f(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d);
			pts_3d_ref.push_back(p_cam);
			pts_3d_all.push_back(p_cam);
			descriptors_ref_.push_back(descriptors_curr_.row(i));
		}
	}

	void VisualOdometry::poseEstimationPnP()
	{
		std::vector<cv::Point3f> pts3d;
		std::vector<cv::Point2f> pts2d;

		//��һ֡�뵱ǰ֡��ƥ���
		for (int i = 0; i < feature_matches_.size(); i++)
		{
			if (pts_3d_ref[feature_matches_[i].queryIdx].z < 0)
				continue;

			//��һ֡����ά��
			pts3d.push_back(pts_3d_ref[feature_matches_[i].queryIdx]);

			//����ά���ڵ�ǰ֡��Ӧ�Ķ�ά��
			pts2d.push_back(keypoints_curr_[feature_matches_[i].trainIdx].pt);
		}

		//�ڲ�
		cv::Mat K = (cv::Mat_<float>(3, 3) << ref_->camera_->fx_, 0.0, ref_->camera_->cx_,
			0.0, ref_->camera_->fy_, ref_->camera_->cy_,
			0.0, 0.0, 1.0);

		cv::Mat rvec = cv::Mat::zeros(3, 1, CV_32FC1);
		cv::Mat tvec = cv::Mat::zeros(3, 1, CV_32FC1);
		std::vector<int> inliers;

		//pnpλ�˼���
		//����û�м�⵽�κ�������
		cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
		num_inliers_ = inliers.size();
		std::cout << "pnp inliers: " << inliers.size() << std::endl;

		cv::Rodrigues(rvec, rvec);

		//��òο�֡����ڵ�ǰ֡��λ�ù�ϵ
		T_c_r_estimate = cv::Mat::eye(4, 4, CV_32FC1);
		rvec.copyTo(T_c_r_estimate(cv::Rect(0, 0, 3, 3)));
		tvec.copyTo(T_c_r_estimate(cv::Rect(3, 0, 1, 3)));
	}

	void VisualOdometry::addKeyFrame()
	{
		std::cout << "adding a key-frame" << std::endl;
		map_->insertKeyFrame(curr_);
	}

	bool VisualOdometry::checkEstimatedPose()
	{
		//�ڵ���̫��
		if (num_inliers_ < min_inliers_)
		{
			std::cout << "reject because inlier is too small: " << num_inliers_ << std::endl;
			return false;
		}

		cv::Mat rvec = T_c_r_estimate(cv::Rect(0, 0, 3, 3));
		cv::Mat tvec = T_c_r_estimate(cv::Rect(3, 0, 1, 3));

		//�ƶ�����̫�������ת����������
		double v2 = abs(1.0 - determinant(rvec));
		float  v1 = tvec.at<float >(0, 0) ;
		
		if (v1 > 20.0 || v2 > 0.01)
		{
			std::cout << "reject because motion is too large: " << std::endl;
			return false;
		}
		return true;
	}

	bool VisualOdometry::checkKeyFrame()
	{
		cv::Mat rvec = T_c_r_estimate(cv::Rect(0, 0, 3, 3));
		cv::Mat tvec = T_c_r_estimate(cv::Rect(3, 0, 1, 3));

		//������ת�Ƕ�
		cv::Scalar t = cv::trace(rvec);
		double trR = t.val[0];
		double theta = acos((trR - 1.0) / 2.0);

		//������С����ת����ƽ����ֵ
		if (abs(theta) > key_frame_min_rot || norm(tvec) > key_frame_min_trans)
			return true;
		return false;
	}
}
