/**********************ROS****************************************/
#include <ros/ros.h>
#include <axis_tf/getPoint.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
/*********************EUGEN***************************************/
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
/********************OPENCVLIBRARY********************************/
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
/*******************PACKAGE_HEADER*******************************/
#include <opencvtest/pixel_point.h>
#include <dobot/mypose.h>
#include <dobot/GetPose.h>
/**************************************计算过程中用到的变量***************************************************/
using namespace std;
axis_tf::getPoint msg;//自定义msg存储的是木块中心的像素坐标
Eigen::Matrix<double, 4, 3 >* pointer_camera_matrix;//相机内参*Zc
ros::NodeHandle* n_p = NULL;
ros::Publisher* pointer_result_1_pub = NULL;
double Zc = 1.0;//坐标变换因子(相机坐标系原点到世界坐标平面的距离)
const string camera_name = "logitech";//相机坐标系的名称
//手动输入标定好的相机内参
cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << 784.672402, 0.000000  , 263.984597,
                                                    0.000000, 785.198744, 240.311843, 
                                                    0.000000, 0.000000  , 1.000000   );
//相机畸变参数
cv::Mat dist_coeffs =(cv::Mat_<double>(1,5) << 0.021417, -0.027116, 0.000933, -0.009602, 0.000000);
//Aruco二维码的编码代号
cv::Ptr<cv::aruco::Dictionary> dictionary;
//二维码中心.即世界坐标系的中心点像素坐标
vector<cv::Point2f> marker_center;
//相机坐标系乘以尺度因子Zc
Eigen::Matrix< double, 4, 3 > camera_matrix_Zc;
//齐次像素坐标初始化,用来储存接收到的木块中心像素
Eigen::Vector3d pixel_vec(1.0, 1.0, 1.0); 
//中心点在相机坐标系下的坐标计算结果
Eigen::Vector4d result_1;    
/**********************************************TF变量******************************************************/
tf::TransformBroadcaster* pointer_marker_position_broadcaster;
tf::TransformBroadcaster* pointer_dobot_base_bro;
tf::TransformBroadcaster* pointer_dobot_effector_bra;
/******************************CALLBACK_FUNCTION*********************/
void callbackCalculateAxis(opencvtest::pixel_point::ConstPtr message);
void callbackImage(const sensor_msgs::ImageConstPtr& msg);
/**********************************FUNCTION**************************/
void loadCalibrationFiles(string& input_path, cv::Mat& camera_matrix, cv::Mat& distcoeffs, double scale);
void getMarker(cv::Mat& marker_image, vector<cv::Point2f>& marker_center, bool key);
//得到世界坐标系的中心点
void getMarkerCoordinate(vector < vector<cv::Point2f> >& corners, vector<int>& ids, vector<cv::Point2f>& marker_center);
void sendMarkerTf(vector<cv::Vec3d>& marker_vecs, vector<cv::Vec3d>& marker_rvecs);
void sendDobotTf();
void sendDobotEffectorTF();
/********************************************************************/
/********************************************************************/
/******************************主函数*********************************/
/********************************************************************/
/********************************************************************/
int main(int argc, char** argv)
{
    
    camera_matrix_Zc<< 0.784672402, 0.000000  , 0.263984597,
                       0.000000,   0.785198744, 0.240311843, 
                       0.000000,    0.000000  , 1.000000, 
                       0.000000,    0.000000  , 1.000000;
    camera_matrix_Zc(0,0) *=  Zc;
    camera_matrix_Zc(0,2) *= -Zc;
    camera_matrix_Zc(1,1) *=  Zc;
    camera_matrix_Zc(1,2) *= -Zc;
    camera_matrix_Zc(2,2) *=  Zc;
    pointer_camera_matrix = &camera_matrix_Zc;
    
    ros::init(argc, argv, "axis_tf");
    ros::NodeHandle n;
    n_p = &n;
    tf::TransformBroadcaster marker_position_broadcaster;
    tf::TransformBroadcaster dobot_base_bro;
    tf::TransformBroadcaster dobot_effector_bra;
    pointer_marker_position_broadcaster = &marker_position_broadcaster;
    pointer_dobot_base_bro = &dobot_base_bro;
    pointer_dobot_effector_bra = &dobot_effector_bra;

    ros::ServiceClient client_pose = n.serviceClient<dobot::GetPose>("dobot/GetPose");//订阅机器人末端的姿态
    image_transport::ImageTransport it_(n);
    image_transport::Subscriber image_sub_= it_.subscribe("/usb_cam/image_raw",1,callbackImage);
    ros::Publisher result_1_pub = n.advertise<axis_tf::getPoint>("result_1", 1000);//发布相机坐标系下的中心坐标
    pointer_result_1_pub = &result_1_pub;
    ros::Subscriber pixel_sub = n.subscribe("pixel_center_axis",100,callbackCalculateAxis);//订阅木块中心的像素坐标
    
    //每秒接受30 次回调函数
    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}

//使用像素坐标计算相机坐标的回调函数
void callbackCalculateAxis(opencvtest::pixel_point::ConstPtr message)
    {           
        if( (pixel_vec[0] != message->u)||(pixel_vec[1] = message->v))
        {
            //计算公式
            pixel_vec[0] = message->u;
            pixel_vec[1] = message->v;
            pixel_vec = pixel_vec.transpose();
            result_1 = (*pointer_camera_matrix) * pixel_vec;
            msg.x1 = result_1[0];
            msg.x2 = result_1[1];
            msg.x3 = result_1[2];
            msg.x4 = result_1[3];
        }
        pointer_result_1_pub->publish(msg);//发布出来
    }
    //接受传来的图像,目的是为了检测二维码,并发布世界坐标系,相机坐标系,机器人坐标系,机器人末端坐标系的tf变换关系
void callbackImage(const sensor_msgs::ImageConstPtr& msg)
    {
        //cout<<"回调函数2！"<<endl;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//转化为opencv格式图像，返回指针
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        //获取二维码信息,并发布坐标系之间的关系
        getMarker(cv_ptr->image,marker_center,0);
        cv::imshow("callbackImage",cv_ptr->image);
        cout<<result_1<<endl;
        cv::waitKey(1); 
    }

void getMarker(cv::Mat& marker_image, vector<cv::Point2f>& marker_center, bool key)
    {
        //从而二维码中获取相机和二维码(世界坐标系)的变化关系
        vector<int> ids;
        vector< vector<cv::Point2f> > corners;
        vector<cv::Vec3d> rvecs, tvecs;
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
        if(!marker_image.empty())
            {
                cv::aruco::detectMarkers(marker_image, dictionary, corners, ids);//侦测到角点以备姿态检测使用
                cv::aruco::drawDetectedMarkers(marker_image, corners, ids);
                cv::aruco::estimatePoseSingleMarkers(corners, 0.117, camera_matrix, dist_coeffs, rvecs, tvecs);//what does the 0.155 mean??????
                if(rvecs.empty()&&tvecs.empty())
                    {
                        cout<<"no trans"<<endl;
                    }
                else
                    {
                        cv::aruco::drawAxis(marker_image, camera_matrix, dist_coeffs ,rvecs, tvecs, 0.1);  
                        //getMarkerCoordinate(corners, ids, marker_center);
                    }  
                //发布tf
                sendMarkerTf(rvecs,tvecs);
            }
    }

void getMarkerCoordinate(vector < vector<cv::Point2f> >& corners, vector<int>& ids, vector<cv::Point2f>& marker_center)
    {
        cv::Point2f center(0.f, 0.f);
        for(int i = 0;i < corners[0].size();i++)
            {
                center += corners[0][i];
            }
        center /= 4.0;
        marker_center.push_back(center);
        cout<<marker_center[0].x<<","<<marker_center[0].y<<endl;
    }

void sendMarkerTf(vector<cv::Vec3d>& marker_rvecs,vector<cv::Vec3d>& marker_tvecs)                 
    {
        if(marker_rvecs.size()==0&&marker_rvecs.size()==0)
            {
                cout<<"haven't received any vecs yet"<<endl;
            }
            else
                {
                    cv::Mat rotated_matrix(3, 3, CV_64FC1);//储存旋转矩阵
                    //tf::TransformBroadcaster marker_position_broadcaster;
                    cv::Rodrigues(marker_rvecs[0],rotated_matrix);//旋转向量转换为旋转矩阵
                    rotated_matrix.convertTo(rotated_matrix, CV_64FC1);
                    tf::Matrix3x3 tf_rotated_matrix(rotated_matrix.at<double>(0,0), rotated_matrix.at<double>(0,1),rotated_matrix.at<double>(0,2),
                                        rotated_matrix.at<double>(1,0), rotated_matrix.at<double>(1,1), rotated_matrix.at<double>(1,2),
                                        rotated_matrix.at<double>(2,0), rotated_matrix.at<double>(2,1), rotated_matrix.at<double>(2,2));

                    tf::Vector3 tf_tvec(marker_tvecs[0][0],marker_tvecs[0][1],marker_tvecs[0][2]);
                    tf::Transform transform(tf_rotated_matrix, tf_tvec);
                    //transform = transform.inverse();
                    //Eigen::Quaterniond q_eigen;
                    //tf::quaternionTFToEigen(transform.getRotation(), q_eigen);
                    //temp_rot = q_eigen;
                    //tf::vectorTFToEigen(transform.getOrigin(), trans);
                    //ostringstream oss;
                    //oss << "camera_" << ids[0];
                    pointer_marker_position_broadcaster->sendTransform(
                        tf::StampedTransform(transform, ros::Time::now(), "logitech", "world"));  
                    //发布机器人和世界坐标系之间的关系
                    sendDobotTf();
        
                }
       
    }
void sendDobotTf()
    { 
        //ros::ServiceClient client_pose_info = n.serviceClient<dobot::GetPose>("dobot/GetPose");
        cv::Mat rotated_matrix(3,3,CV_64FC1);
        rotated_matrix.at<float>(0,0)= 0.0;rotated_matrix.at<float>(0,1)= 1.0;rotated_matrix.at<float>(0,2)= 0.0;
        rotated_matrix.at<float>(1,0)= -1.0;rotated_matrix.at<float>(1,1)= 0.0;rotated_matrix.at<float>(1,2)= 0.0;
        rotated_matrix.at<float>(2,0)= 0.0;rotated_matrix.at<float>(2,1)= 0.0;rotated_matrix.at<float>(2,2)= 1.0;


        tf::Matrix3x3 tf_rotated_matrix(
        rotated_matrix.at<float>(0,0), rotated_matrix.at<float>(0,1), rotated_matrix.at<float>(0,2),
        rotated_matrix.at<float>(1,0), rotated_matrix.at<float>(1,1), rotated_matrix.at<float>(1,2),
        rotated_matrix.at<float>(2,0), rotated_matrix.at<float>(2,1), rotated_matrix.at<float>(2,2));

        //tf::TransformBroadcaster dobot_base_bro;
        tf::Vector3 tf_tvecs(-0.068, 0.265, 0.117);//测量值,机器人基座标系中心和细节坐标系中心的关系
        tf::Transform transform(tf_rotated_matrix, tf_tvecs);
        //cv::Rodrigues();
        //dobot::GetPose srv;
        //client_pose_info.call(srv);
        //tf::Vector3 tf_tvec(srv.response.x, srv.response.y, srv.response.z):
        pointer_dobot_base_bro->sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world","dobot_base"));
        sendDobotEffectorTF();//发布机器人末端坐标系和基座标系的变化关系
    }

void sendDobotEffectorTF()
    {
        ros::ServiceClient client_getpose = n_p->serviceClient<dobot::GetPose>("/DobotServer/GetPose"); 
        cv::Mat rotated_matrix(3,3,CV_64FC1);
        rotated_matrix.at<float>(0,0)= 1.0;rotated_matrix.at<float>(0,1)= 0.0;rotated_matrix.at<float>(0,2)= 0.0;
        rotated_matrix.at<float>(1,0)= 0.0;rotated_matrix.at<float>(1,1)= 1.0;rotated_matrix.at<float>(1,2)= 0.0;
        rotated_matrix.at<float>(2,0)= 0.0;rotated_matrix.at<float>(2,1)= 0.0;rotated_matrix.at<float>(2,2)= 1.0;

        tf::Matrix3x3 tf_rotated_matrix(
        rotated_matrix.at<float>(0,0), rotated_matrix.at<float>(0,1), rotated_matrix.at<float>(0,2),
        rotated_matrix.at<float>(1,0), rotated_matrix.at<float>(1,1), rotated_matrix.at<float>(1,2),
        rotated_matrix.at<float>(2,0), rotated_matrix.at<float>(2,1), rotated_matrix.at<float>(2,2));
    
        //tf::TransformBroadcaster dobot_effector_bra;
        dobot::GetPose srv;
        client_getpose.call(srv);
        tf::Vector3 tf_tvecs(srv.response.x/1000, srv.response.y/1000, srv.response.z/1000);
        tf::Transform transform(tf_rotated_matrix, tf_tvecs);
        pointer_dobot_effector_bra->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "dobot_base", "dobot_effector" ));
        //cout<<"already send effector tf"<<endl;
    }

