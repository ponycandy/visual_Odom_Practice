#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "common_include.h"
#include "map.h"
#include "viewer.h"
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>

namespace myslam
{
    class Viewer;
    class VisualOdometry
    {
    public:
        //��̼�״̬
        enum VOState
        {
            INITIALZING = -1,
            OK = 0,
            LOST
        };

        VOState state_;

        //��ͼ
        Map* map_;

        //ͼ��֡,�ο�֡,��ǰ֡
        Frame* ref_;
        Frame* curr_;

        //���е�ͼ��֡
        std::vector<Frame*> mvAllFrame;

        Viewer* mpViewer;
        Camera* mp_camera;

        cv::Ptr<cv::ORB> orb_;

        //�ο�֡��ά��,����֡��ά��
        std::vector<cv::Point3f> pts_3d_ref;
        std::vector<cv::Point3f> pts_3d_all;

        //��ǰ֡������
        std::vector<cv::KeyPoint> keypoints_curr_;

        //��ǰ֡���ο�֡������
        cv::Mat descriptors_curr_, descriptors_ref_;

        //����ƥ��
        std::vector<cv::DMatch> feature_matches_;

        //�ο�֡����ǰ֡��λ����Ϣ
        cv::Mat T_c_r_estimate;

        //id
        int frameid;
        int mathe_flag;
        //�ڵ�����
        int num_inliers_;
        int num_lost_;

        //orb������ȡ����
        int num_of_features_;
        double scale_factor_;
        int level_pyramid_;
        float match_ratio_;
        int max_num_lost_;
        int min_inliers_;

        //�ؼ�֡����С��ת��ƽ��
        double key_frame_min_rot;
        double key_frame_min_trans;

    public:
        VisualOdometry(std::string strSettingPath, Viewer* pViewer);
        ~VisualOdometry();

        cv::Mat Tracking(cv::Mat im, cv::Mat imD, double tframe);
    private:
        cv::Mat addFrame(Frame* frame);

        void ExtractORB();
        void featureMatching();
        void poseEstimationPnP();
        void setRef3DPoints();

        void addKeyFrame();
        bool checkEstimatedPose();
        bool checkKeyFrame();
    };
}
#endif
