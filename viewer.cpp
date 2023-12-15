#include "viewer.h"

namespace myslam
{
	Viewer::Viewer()
	{
		//nh = new gpcs::gpcsnode();
		//nh->init("Visual_Odom");
		//pub_3Dpoints = nh->advertise("Slam_data/3D_points");
		//pub_Camera_pos = nh->advertise("Slam_data/Camerapos");

		/*cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

		float fps = fSettings["Camera.fps"];
		if (fps < 1)
			fps = 30;
		mT = 1e3 / fps;

		mImageWidth = fSettings["Camera.width"];
		mImageHeight = fSettings["Camera.height"];
		if (mImageWidth < 1 || mImageHeight < 1)
		{
			mImageWidth = 640;
			mImageHeight = 480;
		}

		mViewpointX = fSettings["Viewer.ViewpointX"];
		mViewpointY = fSettings["Viewer.ViewpointY"];
		mViewpointZ = fSettings["Viewer.ViewpointZ"];
		mViewpointF = fSettings["Viewer.ViewpointF"];

		mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
		mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
		mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
		mPointSize = fSettings["Viewer.PointSize"];
		mCameraSize = fSettings["Viewer.CameraSize"];
		mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

		VisualOdometry_ = nullptr;*/
	}

	// pangolin����ĵ���http://docs.ros.org/fuerte/api/pangolin_wrapper/html/namespacepangolin.html
	void Viewer::Run()
	{
		//{
		//	//������,��ʼ����ʾ����ʱ,������λ������,��Ӧvisual_odometry�е�addframe����
		//	std::unique_lock<std::mutex> lock(mMutexViewer);
		//	pangolin::CreateWindowAndBind("my-slam: Map Viewer", 1024, 768);

		//	// ������Ȳ��ԣ�OpenGLֻ������ǰ���һ�㣬����ʱ��鵱ǰ����ǰ���Ƿ��б�����أ����������ص�ס�����������Ͳ������
		//	glEnable(GL_DEPTH_TEST);

		//	// ��OpenGL��ʹ����ɫ���
		//	glEnable(GL_BLEND);

		//	// ѡ����ѡ��
		//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

		//	// �½���ť��ѡ��򣬵�һ������Ϊ��ť�����֣��ڶ���ΪĬ��״̬��������Ϊ�Ƿ���ѡ���
		//	pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
		//	pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
		//	pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
		//	pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
		//	pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
		//	pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
		//	pangolin::Var<bool> menuReset("menu.Reset", false, false);

		//	// �������ͶӰģ�ͣ�ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
		//	// ����۲ⷽλ�������۲��λ�ã�(mViewpointX mViewpointY mViewpointZ)
		//	//                �۲�Ŀ��λ�ã�(0, 0, 0)
		//	//                �۲�ķ�λ������(0.0,-1.0, 0.0)
		//	s_cam = pangolin::OpenGlRenderState(
		//		pangolin::ProjectionMatrix(1024, 768, mViewpointF, mViewpointF, 512, 389, 0.1, 1000),
		//		pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ, 0, 0, 0, 0.0, -1.0, 0.0)
		//	);

		//	Tcw.SetIdentity();
		//	cv::namedWindow("my-slam: Current Frame");

		//	// ������ʾ����С��orbslam��������������壬�����ʾһЩ��ť���ұ���ʾͼ��
		//	// ǰ����������0.0, 1.0��������Ⱥ���������Ⱥʹ��ڴ�С��ͬ
		//	// �м�����������pangolin::Attach::Pix(175), 1.0�������ұ����в���������ʾͼ��
		//	// ���һ��������-1024.0f/768.0f��Ϊ��ʾ�����
		//	d_cam = pangolin::CreateDisplay()
		//		.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
		//		.SetHandler(new pangolin::Handler3D(s_cam));
		//}

		//while (1)
		//{
		//	// ����������еĵ�ǰ��д����ɫ���� �� ��Ȼ���
		//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//	// �õ����µ����λ��
		//	GetCurrentOpenGLCameraMatrix(Tcw);

		//	s_cam.Follow(Tcw);
		//	d_cam.Activate(s_cam);

		//	// ����Ϊ��ɫ��glClearColor(red, green, blue, alpha������ֵ��Χ(0, 1)
		//	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
		//	DrawCurrentCamera(Tcw);
		//	DrawTrackingFrame();
		//	DrawMapPoints();

		//	pangolin::FinishFrame();

		//	cv::Mat im = DrawFrame();
		//	if (im.empty())
		//		continue;

		//	cv::imshow("my-slam: Current Frame", im);
		//	cv::waitKey(10);
		//}
	}

	void Viewer::DrawTrackingFrame()
	{
		//const float& w = mKeyFrameSize * 0.3;
		//const float h = w * 0.75;
		//const float z = w * 0.6;

		//{
		//	//������,��ʹ�ùؼ�֡��Ϣʱ,�����޸�
		//	std::unique_lock<std::mutex> lock(mMutexTrackingFrame);

		//	//�������еĹؼ�֡λ��
		//	for (int i = 0; i < mvAllFrame.size(); i++)
		//	{
		//		Frame* cur = mvAllFrame[i];
		//		cv::Mat Twc = cur->T_c_w_.inv();

		//		pangolin::OpenGlMatrix M;
		//		if (!Twc.empty())
		//		{
		//			cv::Mat Rwc(3, 3, CV_32F);
		//			cv::Mat twc(3, 1, CV_32F);
		//			{
		//				Rwc = Twc.rowRange(0, 3).colRange(0, 3);
		//				twc = Twc.rowRange(0, 3).col(3);
		//			}

		//			M.m[0] = Rwc.at<float>(0, 0);
		//			M.m[1] = Rwc.at<float>(1, 0);
		//			M.m[2] = Rwc.at<float>(2, 0);
		//			M.m[3] = 0.0;

		//			M.m[4] = Rwc.at<float>(0, 1);
		//			M.m[5] = Rwc.at<float>(1, 1);
		//			M.m[6] = Rwc.at<float>(2, 1);
		//			M.m[7] = 0.0;

		//			M.m[8] = Rwc.at<float>(0, 2);
		//			M.m[9] = Rwc.at<float>(1, 2);
		//			M.m[10] = Rwc.at<float>(2, 2);
		//			M.m[11] = 0.0;

		//			M.m[12] = twc.at<float>(0);
		//			M.m[13] = twc.at<float>(1);
		//			M.m[14] = twc.at<float>(2);
		//			M.m[15] = 1.0;
		//		}
		//		else
		//			M.SetIdentity();

		//		glPushMatrix();
		//		glMultMatrixd(M.m);

		//		glLineWidth(mKeyFrameLineWidth);
		//		glColor3f(0.0f, 0.0f, 1.0f);
		//		glBegin(GL_LINES);
		//		glVertex3f(0, 0, 0);
		//		glVertex3f(w, h, z);
		//		glVertex3f(0, 0, 0);
		//		glVertex3f(w, -h, z);
		//		glVertex3f(0, 0, 0);
		//		glVertex3f(-w, -h, z);
		//		glVertex3f(0, 0, 0);
		//		glVertex3f(-w, h, z);

		//		glVertex3f(w, h, z);
		//		glVertex3f(w, -h, z);

		//		glVertex3f(-w, h, z);
		//		glVertex3f(-w, -h, z);

		//		glVertex3f(-w, h, z);
		//		glVertex3f(w, h, z);

		//		glVertex3f(-w, -h, z);
		//		glVertex3f(w, -h, z);
		//		glEnd();

		//		glPopMatrix();
		//	}
		//}
	}

	void Viewer::DrawMapPoints()
	{
		//{
		//	//������,��ʹ����ά����Ϣʱ,�����޸�
		//	std::unique_lock<std::mutex> lock(mMutex3dPoints);
		//	glPointSize(mPointSize);
		//	glBegin(GL_POINTS);
		//	glColor3f(0.0, 0.0, 0.0);

		//	//�������е���ά��
		//	for (int i = 0; i < mAll3dpts.size(); i += 10)
		//	{
		//		cv::Mat pos = cv::Mat(mAll3dpts[i]);
		//		glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
		//	}
		//	glEnd();
		//}
	}

	//cv::Mat Viewer::DrawFrame()
	//{
	//	//const float r = 5;

	//	//if (VisualOdometry_ == nullptr)
	//	//	return cv::Mat();

	//	//if (VisualOdometry_->keypoints_curr_.size() == 0 || VisualOdometry_->feature_matches_.size() == 0)
	//	//	return cv::Mat();

	//	//{
	//	//	//������,��ʹ����̼���Ϣʱ,�����޸�
	//	//	std::unique_lock<std::mutex> lock(mMutex);

	//	//	mkps = VisualOdometry_->keypoints_curr_;
	//	//	mfeature_matches = VisualOdometry_->feature_matches_;
	//	//	mIm = VisualOdometry_->curr_->color_;

	//	//	n = mfeature_matches.size();
	//	//}

	//	////��ͼ���ϻ��Ƴ���ά������
	//	//for (int i = 0; i < n; i++)
	//	//{
	//	//	int index = mfeature_matches[i].trainIdx;
	//	//	cv::Point2f pt1, pt2;
	//	//	pt1.x = mkps[index].pt.x - r;
	//	//	pt1.y = mkps[index].pt.y - r;
	//	//	pt2.x = mkps[index].pt.x + r;
	//	//	pt2.y = mkps[index].pt.y + r;

	//	//	cv::rectangle(mIm, pt1, pt2, cv::Scalar(0, 255, 0));
	//	//	cv::circle(mIm, mkps[index].pt, 2, cv::Scalar(0, 255, 0), -1);
	//	//}

	//	//return mIm;
	//}

	void Viewer::SetCurrentCameraPose(const cv::Mat& Tcw)
	{
		////������,��ʹ�����λ��ʱ,�����޸���ֵ
		//std::unique_lock<std::mutex> lock(mMutexCamera);
		//mCameraPose = Tcw.clone();
		//mCameraPose = mCameraPose.inv();
	}

	void Viewer::SetVisualOdometry(myslam::VisualOdometry* pVisualOdometry)
	{
		//������,��ʹ����̼���Ϣʱ,�����޸�
		//std::unique_lock<std::mutex> lock(mMutex);
		//VisualOdometry_ = pVisualOdometry;
	}

	void Viewer::GetAllFrame(std::vector<Frame*> vAllFrame)
	{
		////������,��ʹ�ùؼ�֡��Ϣʱ,�����޸�
		//std::unique_lock<std::mutex> lock(mMutexTrackingFrame);
		//mvAllFrame = vAllFrame;
	}

	void Viewer::GetAll3dPoints(std::vector<cv::Point3f> All3dpts)
	{
		////������,��ʹ����ά����Ϣʱ,�����޸�
		//std::unique_lock<std::mutex> lock(mMutex3dPoints);
		//mAll3dpts = All3dpts;
	}

	void Viewer::Publish()
	{
		//�õģ���Щ������Ҫ�㲥�أ��ο�һ������Ŀ��ӻ�����
		//���λ�ã������ʷ���йؼ�֡λ�ã�������ʷIMD������λ��
		//��ʷλ�ò��õ������ӻ��ƣ�ÿ��ֻ�㲥һ��λ�ã�������Ϊ0���Ǿ���û���µĹؼ�֡��
		//������������publisher���ֱ�Ϊ�����pos��publisher,ÿ�������µ�λ�õ�ʱ��㲥
		//�ؼ�֡λ�ã�ͬ��,��ʵ����û�г���lost��ʱ���ÿһ֡���λ��
		//������λ��ͬ�ϣ����ǵ��3Dλ�ã�����û����ɫ
		//��ô,publisher�����ûɶ��Ҫ��
		//����pos��4*4�������ֱ�Ӵ���
		//����ͨ�����ݷ��͸�ʽ������
	}

	//	//����gl��صĺ�������ֱ��google, ������msdn�ؼ���
	//	//void Viewer::DrawCurrentCamera(pangolin::OpenGlMatrix& Twc)
	//	{
	//		////���ģ�ʹ�С
	//		//const float& w = mCameraSize * 0.5;
	//		//const float h = w * 0.75;
	//		//const float z = w * 0.6;
	//
	//		//glPushMatrix();
	//
	//		////��4*4�ľ���Twc.m�ҳ�һ����ǰ����
	//		////������ʹ����glPushMatrix��������˵�ǰ֡����Ϊ��������ϵ�µĵ�λ����
	//		////��ΪOpenGL�еľ���Ϊ�����ȴ洢�����ʵ��ΪTcw������������������µ�λ��
	//		//glMultMatrixd(Twc.m);
	//
	//		////���û���ͼ��ʱ�ߵĿ��
	//		//glLineWidth(mCameraLineWidth);
	//
	//		////���õ�ǰ��ɫΪ��ɫ(���ͼ����ʾΪ��ɫ)
	//		//glColor3f(0.0f, 1.0f, 0.0f);
	//
	//		////���߽�����Ķ�����������
	//		//glBegin(GL_LINES);
	//		//glVertex3f(0, 0, 0);
	//		//glVertex3f(w, h, z);
	//		//glVertex3f(0, 0, 0);
	//		//glVertex3f(w, -h, z);
	//		//glVertex3f(0, 0, 0);
	//		//glVertex3f(-w, -h, z);
	//		//glVertex3f(0, 0, 0);
	//		//glVertex3f(-w, h, z);
	//
	//		//glVertex3f(w, h, z);
	//		//glVertex3f(w, -h, z);
	//
	//		//glVertex3f(-w, h, z);
	//		//glVertex3f(-w, -h, z);
	//
	//		//glVertex3f(-w, h, z);
	//		//glVertex3f(w, h, z);
	//
	//		//glVertex3f(-w, -h, z);
	//		//glVertex3f(w, -h, z);
	//		//glEnd();
	//
	//		//glPopMatrix();
	//	}
	//
	//	// �����λ��mCameraPose��Mat����ת��ΪOpenGlMatrix����
	////	void Viewer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix& M)
	////	{
	////	//	if (!mCameraPose.empty())
	////	//	{
	////	//		cv::Mat Rcw(3, 3, CV_32F);
	////	//		cv::Mat tcw(3, 1, CV_32F);
	////	//		{
	////	//			//������,��ʹ�����λ��ʱ,�����޸���ֵ
	////	//			std::unique_lock<std::mutex> lock(mMutexCamera);
	////	//			Rcw = mCameraPose.rowRange(0, 3).colRange(0, 3);
	////	//			tcw = mCameraPose.rowRange(0, 3).col(3);
	////	//		}
	////
	////	//		M.m[0] = Rcw.at<float>(0, 0);
	////	//		M.m[1] = Rcw.at<float>(1, 0);
	////	//		M.m[2] = Rcw.at<float>(2, 0);
	////	//		M.m[3] = 0.0;
	////
	////	//		M.m[4] = Rcw.at<float>(0, 1);
	////	//		M.m[5] = Rcw.at<float>(1, 1);
	////	//		M.m[6] = Rcw.at<float>(2, 1);
	////	//		M.m[7] = 0.0;
	////
	////	//		M.m[8] = Rcw.at<float>(0, 2);
	////	//		M.m[9] = Rcw.at<float>(1, 2);
	////	//		M.m[10] = Rcw.at<float>(2, 2);
	////	//		M.m[11] = 0.0;
	////
	////	//		M.m[12] = tcw.at<float>(0);
	////	//		M.m[13] = tcw.at<float>(1);
	////	//		M.m[14] = tcw.at<float>(2);
	////	//		M.m[15] = 1.0;
	////	//	}
	////	//	else
	////	//		M.SetIdentity();
	////	//}
	////}
}