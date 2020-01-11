/*********************** opencv *************************/
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
/*********************** Marker *************************/
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <vector>
/*********************** ROS ****************************/
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>

/*********************** LIBRARIES **********************/
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
/*********************** ROSBAG *************************/
#include <openpose_ros_msgs/OpenPoseHumanList.h>
#include <openpose_ros_msgs/PointWithProb.h>

#include <iostream>
#include <sstream>
#include <fstream>
 
using namespace std;
using namespace cv;
using namespace ceres;

struct KeyPoint_prob{
	double x;
	double y;
	double p;
};

class ImageProcessor
{
public:
	ImageProcessor(int camera_index):it(nh),sizeColor(660,360)
	{
		cout << "回调函数开始" << endl;
		camera_index_ = camera_index;
		initCalibration(0.333,camera_index_);
		image = Mat(sizeColor,CV_8UC3);
		human_joint_names.push_back("/hip");

		char color_topic[50];
		sprintf(color_topic,"kinect2_%d/qhd/image_color_rect",camera_index);
		image_sub =it.subscribe(color_topic,1,&ImageProcessor::imageCallback,this);
		
		char key_points_topic[50];
		sprintf(key_points_topic,"openpose_ros/human_list_%d",camera_index);
		human_keypoints_sub = nh.subscribe(key_points_topic,1,&ImageProcessor::human_keypoints_callback,this);
		
		cout << "回调函数结束" << endl;
	}

	void getMarkerCenter(vector<Point2f>& marker_center)
	{
		marker_center = this->marker_center;
	}

	void getCameraRot(Eigen::Matrix3d & camera_rot)
	{
		camera_rot = this->camera_rot;
	}

	void getCameraTrans(Eigen::Matrix<double,3,1>& camera_trans)
	{
		camera_trans = this->camera_trans;
	}

	void getKinectPose(Eigen::Matrix3d &kinect_rot, Eigen::Matrix<double,3,1> &kinect_trans)
	{
		kinect_rot = this->kinect_rot;
		kinect_trans = this->kinect_trans;
	}

	void getHumanJoint(vector<KeyPoint_prob>& key_points_basic)
	{
		key_points_basic = this->key_points_basic;
	}

	void sendMarkerTf(vector<Vec3d>& marker_trans,vector<Vec3d>& marker_rot,vector<int>& ids,string camera_id,Eigen::Matrix3d& temp_rot,Eigen::Matrix<double,3,1>& trans);
	//void sendWorldTf(const Eigen::Matrix<double, 3, 1>& point,const int camera_id, const string& camera_name);
	void sendWorldTf(const Point3d& point,const int axes_id, const string& camera_name);

/************************************* Camera **********************************************/
	
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void loadCalibrationFiles(std::string& calib_path, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, double scale);
	void initCalibration(double scale,int camera_index);
	void ComputeKinectPose(Eigen::Matrix3d &Cam_rot, Eigen::Matrix<double,3,1> &Cam_trans);

/************************************* Marker function *************************************/

	void getMarker(cv::Mat &marker_image,vector<Point2f>& marker_center, bool Key);
	void getMarkerCoordinate(vector<vector<Point2f>>& corners,vector<int>& ids,vector<Point2f>& marker_center_);

/************************************* human_track *****************************************/

	void human_keypoints_callback(openpose_ros_msgs::OpenPoseHumanList keypoints);
	void draw_human_pose(Mat& image,int Point_num, Scalar color);

private:
	int camera_index_;//相机索引

	ros::NodeHandle nh;
	image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
	ros::Subscriber human_keypoints_sub;

	tf::TransformListener listener;

	cv::Mat distCoeffs;//畸变参数
	cv::Mat cameraMatrix;//相机内参

	cv::Mat image;
	cv::Size sizeColor;

	cv::Ptr<aruco::Dictionary> dictionary;
	
	Eigen::Matrix3d camera_rot;
	Eigen::Matrix<double,3,1> camera_trans;
	Eigen::Matrix3d kinect_rot;
	Eigen::Matrix<double,3,1> kinect_trans;
	
	vector<Point2f> marker_center;
	Eigen::Matrix<double,3,1> worldPoint;

	vector<std::string> human_joint_names;
	vector<KeyPoint_prob> key_points_basic;


};


void ImageProcessor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv::Mat color_mat = cv_bridge::toCvShare(msg,"bgr8")->image;
		image = color_mat.clone();
		ComputeKinectPose(kinect_rot,kinect_trans);
		draw_human_pose(image,7,cv::Scalar(255,0,0));


		getMarker(image,marker_center,0);
		//cout << "marker_center:" << marker_center << endl;
		char camera_ID[20];
		sprintf(camera_ID,"camera_%d",camera_index_);
		cv::imshow(camera_ID,image);
		cv::waitKey(3);
		
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s",e.what());
		return;
	}
}


void ImageProcessor::loadCalibrationFiles(std::string& calib_path, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, double scale)
{
	cv::FileStorage fs;

	cv::Mat cameraMatrix_origin;

	calib_path = calib_path + "/.yaml";
	std::cout << calib_path << std::endl;
	if(fs.open(calib_path, cv::FileStorage::READ))
	{
		std::cout << "open!" << std::endl;
		fs["cameraMatrix"] >> cameraMatrix_origin;
		std::cout << "matrix load success" << std::endl;
		cameraMatrix = cameraMatrix_origin.clone();
		cameraMatrix.at<double>(0,0) *=scale;
		cameraMatrix.at<double>(1,1) *=scale;
		cameraMatrix.at<double>(0,2) *=scale;
		cameraMatrix.at<double>(1,2) *=scale;

		distCoeffs = cv::Mat::zeros(1,5,CV_64F);

		//fs["distortionCoefficients"] >> distCoeffs;
		//std::cout << "matrix load success" << std::endl;
		fs.release();
	}
}


void ImageProcessor::initCalibration(double scale,int camera_index)
{
	string calib_path = "/home/xuchengjun/catkin_ws/src/cv_camera/kinect_calibration_data";
	loadCalibrationFiles(calib_path,cameraMatrix,distCoeffs,scale);
}


void ImageProcessor::getMarkerCoordinate(vector<vector<Point2f>>& corners,vector<int>& ids,vector<Point2f>& marker_center_)
{
	for(int i=0;i<ids.size();i++)
	{
		Point2f center(0.f,0.f);
		for(int j=0;j<corners[i].size();j++)
		{
			if(ids[i] == 0)
			{
				center += corners[i][j];
			}
		}
		center /= 4.0;
		marker_center_.push_back(center);
	}
}


void ImageProcessor::getMarker(cv::Mat &marker_image,vector<Point2f>& marker_center, bool Key)
{
	dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f> > corners;
	std::vector<cv::Vec3d> rvecs,tvecs;
	
	if(!marker_image.empty())
	{
		cv::aruco::detectMarkers(marker_image,dictionary,corners,ids);
		if(ids.size() > 0)
		{
			cv::aruco::drawDetectedMarkers(marker_image,corners,ids);
			for(int i=0;i<ids.size();i++)
			{
				cv::aruco::estimatePoseSingleMarkers(corners,0.155,cameraMatrix,distCoeffs,rvecs,tvecs);
				if(!rvecs.empty() && !tvecs.empty() && ids[i] == 0)
				{
					cv::aruco::drawAxis(marker_image,cameraMatrix,distCoeffs,rvecs[i],tvecs[i],0.1);
					getMarkerCoordinate(corners,ids,marker_center);
					
				}
				char num[10];
				sprintf(num,"camera_%d",camera_index_);
				if(Key == 1)
				{
				sendMarkerTf(tvecs,rvecs,ids,num,camera_rot,camera_trans);
				}
			}
		}
	}
	
}


void ImageProcessor::sendMarkerTf(vector<Vec3d>& marker_trans,vector<Vec3d>& marker_rot,vector<int>& ids,string camera_id,Eigen::Matrix3d& temp_rot,Eigen::Matrix<double,3,1>& trans)
{
	Mat rot(3, 3, CV_64FC1);
	Mat rot_to_ros(3, 3, CV_64FC1);
	rot_to_ros.at<double>(0,0) = -1.0;
	rot_to_ros.at<double>(0,1) = 0.0;
	rot_to_ros.at<double>(0,2) = 0.0;
	rot_to_ros.at<double>(1,0) = 0.0;
	rot_to_ros.at<double>(1,1) = 0.0;
	rot_to_ros.at<double>(1,2) = 1.0;
	rot_to_ros.at<double>(2,0) = 0.0;
	rot_to_ros.at<double>(2,1) = 1.0;
	rot_to_ros.at<double>(2,2) = 0.0;

	static tf::TransformBroadcaster marker_position_broadcaster;
    for(int i = 0; i < ids.size(); i++)
    {
        
		if(ids[i] == 0)
		{
			cv::Rodrigues(marker_rot[i], rot);
        	rot.convertTo(rot, CV_64FC1);

		
        	tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                             rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                             rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));
			
			
        	tf::Vector3 tf_trans(marker_trans[i][0], marker_trans[i][1], marker_trans[i][2]);
        	tf::Transform transform(tf_rot, tf_trans);
			transform = transform.inverse();

			
			Eigen::Quaterniond q_eigen;
			tf::quaternionTFToEigen(transform.getRotation(), q_eigen);
			temp_rot = q_eigen;
			tf::vectorTFToEigen(transform.getOrigin(), trans);
        	ostringstream oss;
        	oss << "marker_" << ids[i];
        	marker_position_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), oss.str(), camera_id.c_str()));
		}
        
    }
}

/*
void ImageProcessor::sendWorldTf(const Eigen::Matrix<double, 3, 1>& point,const int camera_id, const string& camera_name)
{
    static tf::TransformBroadcaster world_position;

    tf::Vector3 tf_trans(point(0,0), point(1,0), point(2,0));
	tf::Quaternion q(0,0,0,1);
    tf::Transform transform(q, tf_trans);
    char marker_name[20];
	sprintf(marker_name, "point_under_cam_%d", camera_id);
    world_position.sendTransform(tf::StampedTransform(transform, ros::Time::now(),camera_name.c_str(), marker_name));
}
*/


void ImageProcessor::sendWorldTf(const Point3d& point,const int axes_id, const string& camera_name)
{
    static tf::TransformBroadcaster world_position;

    tf::Vector3 tf_trans(point.x, point.y, point.z);
	tf::Quaternion q(0,0,0,1);
    tf::Transform transform(q, tf_trans);
    char marker_name[20];
	sprintf(marker_name, "%d", axes_id);
    world_position.sendTransform(tf::StampedTransform(transform, ros::Time::now(),camera_name.c_str(), marker_name));
}


void ImageProcessor::ComputeKinectPose(Eigen::Matrix3d &Cam_rot, Eigen::Matrix<double,3,1> &Cam_trans)
{
	tf::StampedTransform transform;
	char kinect_id[10];
	sprintf(kinect_id,"camera_base_%d",camera_index_);
 	try
    {
        listener.waitForTransform("/marker_0",kinect_id,ros::Time(0),ros::Duration(3.0));
        listener.lookupTransform("/marker_0",kinect_id,ros::Time(0),transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
	Eigen::Quaterniond q(transform.getRotation().getW(),transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ());
    Eigen::Vector3d trans(transform.getOrigin().getX(),transform.getOrigin().getY(),transform.getOrigin().getZ());
	Cam_rot=q.toRotationMatrix();
	Cam_trans = trans;
}



void ImageProcessor::human_keypoints_callback(openpose_ros_msgs::OpenPoseHumanList keypoints)
	{
		key_points_basic.clear();
		int person_num = keypoints.num_humans;
		vector<vector<double> > distance_pool;
		distance_pool.resize(person_num);
		if(person_num > 0)
		{
			int person_index = 0;
			for(int person=0;person < person_num;++person)
			{
				auto body_keypoints = keypoints.human_list[person].body_key_points_with_prob;

				int count = 0;
				double prob_sum = 0.0;
				for(int i=0;i < body_keypoints.size();i++)
				{
					if(body_keypoints[i].prob > 0.0)
					{
						prob_sum += body_keypoints[i].prob;
						count++;
					}
				}
				double prob_eval = prob_sum/count;
				if(prob_eval < 0.4)
				{
					continue;
				}

				KeyPoint_prob key_point_ite;
				//hip
				key_point_ite.x = body_keypoints[8].x;
				key_point_ite.y = body_keypoints[8].y;
				key_point_ite.p = body_keypoints[8].prob;
				key_points_basic.push_back(key_point_ite);
				//rArm
				key_point_ite.x = body_keypoints[2].x;
				key_point_ite.y = body_keypoints[2].y;
				key_point_ite.p = body_keypoints[2].prob;
				key_points_basic.push_back(key_point_ite);
				//rElbow
				key_point_ite.x = body_keypoints[3].x;
				key_point_ite.y = body_keypoints[3].y;
				key_point_ite.p = body_keypoints[3].prob;
				key_points_basic.push_back(key_point_ite);
				//rWrist
				key_point_ite.x = body_keypoints[4].x;
				key_point_ite.y = body_keypoints[4].y;
				key_point_ite.p = body_keypoints[4].prob;
				key_points_basic.push_back(key_point_ite);
				//lArm
				key_point_ite.x = body_keypoints[5].x;
				key_point_ite.y = body_keypoints[5].y;
				key_point_ite.p = body_keypoints[5].prob;
				key_points_basic.push_back(key_point_ite);
				//lElbow
				key_point_ite.x = body_keypoints[6].x;
				key_point_ite.y = body_keypoints[6].y;
				key_point_ite.p = body_keypoints[6].prob;
				key_points_basic.push_back(key_point_ite);
				//lWrist
				key_point_ite.x = body_keypoints[7].x;
				key_point_ite.y = body_keypoints[7].y;
				key_point_ite.p = body_keypoints[7].prob;
				key_points_basic.push_back(key_point_ite);
			}
		}
	}

void ImageProcessor::draw_human_pose(Mat& image,int Point_num, Scalar color)
{
	if(!key_points_basic.empty())
	{
		for(int i=0;i < Point_num;i++)
		{
			circle(image,cv::Point(key_points_basic[i].x, key_points_basic[i].y),4,color,-1,8);
		}
	}
}


struct CostFunction_cam {
  	CostFunction_cam(Point2d _observe_point1, Point2d _observe_point2, 
	  				  Eigen::Matrix3d _camera_rot_1, Eigen::Matrix3d _camera_rot_2,
					  Eigen::Matrix<double,3,1> _camera_trans_1,Eigen::Matrix<double,3,1> _camera_trans_2)
	  :observe_point1(_observe_point1),observe_point2(_observe_point2),
	   camera_rot_1(_camera_rot_1), camera_rot_2(_camera_rot_2),
	   camera_trans_1(_camera_trans_1), camera_trans_2(_camera_trans_2){}

// 残差的计算
  	template <typename T> 
	  bool operator()(const T *const mn, T *residual) const {
		const T cx = (T)(959.19/2), cy = (T)(578.16/2), fx = (T)(1061.45/2), fy = (T)(1056.70/2);
		
		Eigen::Matrix<T,3,1> Cam_point1;
		Eigen::Matrix<T,3,1> Cam_point2;
		Eigen::Matrix<T,3,1> World_point1;
		Eigen::Matrix<T,3,1> World_point2;
		
		Cam_point1(2,0) = mn[0];
		Cam_point2(2,0) = mn[1];
		Cam_point1(0,0) = (T(observe_point1.x) - cx) * Cam_point1(2,0) / fx;
		Cam_point1(1,0) = (T(observe_point1.y) - cy) * Cam_point1(2,0) / fy;
		Cam_point2(0,0) = (T(observe_point2.x) - cx) * Cam_point2(2,0) / fx;
		Cam_point2(1,0) = (T(observe_point2.y) - cy) * Cam_point2(2,0) / fy;
		World_point1 = camera_rot_1.cast<T>() * Cam_point1 + camera_trans_1.cast<T>();
		World_point2 = camera_rot_2.cast<T>() * Cam_point2 + camera_trans_2.cast<T>();

    	residual[0] = (World_point1(0,0) - World_point2(0,0)) * (World_point1(0,0) - World_point2(0,0))
					+(World_point1(1,0) - World_point2(1,0)) * (World_point1(1,0) - World_point2(1,0))
					+(World_point1(2,0) - World_point2(2,0)) * (World_point1(2,0) - World_point2(2,0));
    	return true;

  	}
	int point_index;
	Point2d observe_point1;
	Point2d observe_point2;
	Eigen::Matrix3d camera_rot_1;
	Eigen::Matrix3d camera_rot_2;
	Eigen::Matrix<double,3,1> camera_trans_1;
	Eigen::Matrix<double,3,1> camera_trans_2;
	
	
};



int main(int argc,char **argv) 
{
	ros::init(argc,argv,"marker_node");
	tf::TransformListener listener;

	ImageProcessor imageprocess_1(1);
	ImageProcessor imageprocess_2(2);
	
	ros::Rate loop_rate(30);

	//vector<Point2f> key_point1, key_point2;
	vector<KeyPoint_prob> key_point1, key_point2;
	Eigen::Matrix3d cam_rot_1, cam_rot_2;
	Eigen::Matrix<double,3,1> cam_trans_1, cam_trans_2;
	vector<Point2d> p1_data(10), p2_data(10);
	
	//配置求解器
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	//options.max_num_iterations = 20; //默认50
	options.minimizer_progress_to_stdout = true;

	int N = 7; //数据点
	//double z1 = 5,z2 = 5,z3 = 5,z4 = 5,z5 = 5,z6 = 5,z7 = 5;        // 估计参数值
  	double mn[8][2] = {};
	double cx = 959.19/2, cy = 578.16/2, fx = 1061.45/2, fy = 1056.70/2;
	
	
	while(ros::ok()){
	
		//imageprocess_1.getMarkerCenter(key_point1);
		//imageprocess_2.getMarkerCenter(key_point2);

		imageprocess_1.getHumanJoint(key_point1);
		imageprocess_2.getHumanJoint(key_point2);

		imageprocess_1.getKinectPose(cam_rot_1,cam_trans_1);
		imageprocess_2.getKinectPose(cam_rot_2,cam_trans_2);

		/*	
		imageprocess_1.getCameraRot(cam_rot_1);
		imageprocess_2.getCameraRot(cam_rot_2);
		imageprocess_1.getCameraTrans(cam_trans_1);
		imageprocess_2.getCameraTrans(cam_trans_2);
		*/
		if(!key_point1.empty() && !key_point2.empty())
		{
			for(int i=0;i<N;i++)
			{
				p1_data[i].x = key_point1[i].x;
				p1_data[i].y = key_point1[i].y;
				p2_data[i].x = key_point2[i].x;
				p2_data[i].y = key_point2[i].y;
				//cout << p1_data[0] << "  " << p2_data[0] << endl;
			}
		}
		//cout << p1_data[1] << "  " << p2_data[1] << endl;
		
		ceres::Problem problem;
		for(int i=0;i < N;i++)
		{
    		problem.AddResidualBlock(     // 向问题中添加误差项
    	  	// 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
      		new ceres::AutoDiffCostFunction<CostFunction_cam, 1, 2>(
        		new CostFunction_cam(p1_data[i], p2_data[i],cam_rot_1,cam_rot_2,cam_trans_1,cam_trans_2)
      		),
      		nullptr,            // 核函数，这里不使用，为空
      		mn[i]                // 待估计参数
    		);
		}
		ceres::Solver::Summary summary;
		ceres::Solve(options, &problem, &summary);
		cout << summary.BriefReport() << endl;
		//cout << mn[1][0] << "  " << mn[1][1] << endl;
		//cout << mn[2][0] << "  " << mn[2][1] << endl;
		
		vector<Point3d> Cam_Point1(10), Cam_Point2(10);
		//Eigen::Matrix<double,3,1> Cam_point1;
		//Eigen::Matrix<double,3,1> Cam_point2;
		//double cx = (959.19/3), cy = (578.16/3), fx = (1061.45/3), fy = (1056.70/3);
		for(int p=0;p<N;p++)
		{
			Cam_Point1[p].z = mn[p][0];
			Cam_Point2[p].z = mn[p][1];
			Cam_Point1[p].x = (p1_data[p].x - cx) * Cam_Point1[p].z / fx;
			Cam_Point1[p].y = (p1_data[p].y - cy) * Cam_Point1[p].z / fy;
			Cam_Point2[p].x = (p2_data[p].x - cx) * Cam_Point2[p].z / fx;
			Cam_Point2[p].y = (p2_data[p].y - cy) * Cam_Point2[p].z / fy;
		}

		//cout << "Cam_point1=\n" << Cam_Point1[0] << endl;
		//cout << "Cam_point2=\n" << Cam_Point2[0] << endl;
		//cout << "Cam_point1=\n" << Cam_Point1[5] << endl;
		//cout << "Cam_point2=\n" << Cam_Point2[5] << endl;

		imageprocess_1.sendWorldTf(Cam_Point1[4],1,"camera_base_1");
		imageprocess_2.sendWorldTf(Cam_Point2[4],2,"camera_base_2");
		//imageprocess_1.sendWorldTf(Cam_Point1[5],3,"camera_base_1");
		//imageprocess_2.sendWorldTf(Cam_Point2[5],4,"camera_base_2");
		imageprocess_1.sendWorldTf(Cam_Point1[6],5,"camera_base_1");
		imageprocess_2.sendWorldTf(Cam_Point2[6],6,"camera_base_2");
		//imageprocess_1.sendWorldTf(Cam_Point1[3],7,"camera_base_1");
		//imageprocess_2.sendWorldTf(Cam_Point2[3],8,"camera_base_2");
	

		//imageprocess_1.sendWorldTf(Cam_point1,1,"camera_base_1");
		//imageprocess_2.sendWorldTf(Cam_point2,2,"camera_base_2");
		
    	ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

