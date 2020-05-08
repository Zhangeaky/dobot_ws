/*********************************ROS***********************************************/
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>//将ROS格式编码的
#include<image_transport/image_transport.h>//包含传输图像的节点
#include<sensor_msgs/image_encodings.h>
/**********************************OPENCVLIBRARIES***********************************/
#include<opencv2/core/core.hpp>//opencv核心数据和算法
#include<opencv2/imgproc/imgproc.hpp>//新式C++风格图像处理函数
#include<opencv2/highgui/highgui.hpp>//C++风格的显示、滑动条鼠标及输入输出相关
#include<opencvtest/pixel_point.h>// 自定义数据,将识别到的点的像素坐标发布出来
#include<iostream>
using namespace std;

static const string OPENCV_WINDOW = "Image window";
static const string OPENCV_WINDOW_1 = "binary";
cv::Mat output_gray;
cv::Mat binary;
cv::Mat mask;

vector< vector<cv::Point> > contours;
vector<cv::Vec4i> hierarcy;

int num_contours = contours.size();
static const int hmin = 45;//35
static const int hmax = 77;

static const int smin = 50;//43
static const int smax = 255;

static const int vmin = 55;
static const int vmax = 255;


class ImageConverter
{
private:
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;//订阅ROS原生图像数据
ros::Publisher center_point_pub_;//用来发布像素坐标
 
public:
ImageConverter()
: it_(nh_)// 初始化列表//
{
image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,&ImageConverter::imageCb, this);//订阅节点，该节点在USB_cam/image_raw中发布。
center_point_pub_= nh_.advertise<opencvtest::pixel_point>("pixel_center_axis",1000);//话题名称和接受队列缓存消息条数；
cv::namedWindow(OPENCV_WINDOW);
}
 
~ImageConverter()
{
cv::destroyWindow(OPENCV_WINDOW);
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)//传入ROS格式的图片指针
  {
      cv_bridge::CvImagePtr cv_ptr;//创建opencv格式图像指针。
      //使用try-catch块如果转换出错会抛出相应的异常
      try
      {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//转化为opencv格式图像，返回指针
      }
      catch (cv_bridge::Exception& e)
      {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
      }
      cv::Mat drawmap = cv_ptr->image.clone(); //将物块的矩形轮廓画在drawmap上。
      cv::cvtColor(cv_ptr->image,cv_ptr->image,CV_BGR2HSV);// 转化为灰度图
      cv::inRange(cv_ptr->image,cv::Scalar(hmin,smin,vmin),cv::Scalar(hmax,smax,vmax),cv_ptr->image);//hsv阈值分割
      cv::medianBlur(cv_ptr->image,cv_ptr->image,25);//中值滤波
      binary = cv_ptr->image.clone();
      
      cv::findContours(cv_ptr->image,contours, hierarcy,
                      CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
      cout<<"we detected "<<contours.size()<<"contour(s)"<<endl;
      vector<cv::Rect> rect;
      for(int i = 0;i <contours.size(); i++)
        {
            rect.push_back(cv::boundingRect(contours[i]));//这是直立矩形,需要你自己找到最小矩形的函数
            if(rect[i].area()>1500)
              {
                cv::rectangle(drawmap, rect[i],cv::Scalar(0,0,255),3);
              }
            opencvtest::pixel_point msgs;
            msgs.u = 0.5*(rect[i].tl().x+rect[i].br().x);
            msgs.v = 0.5*(rect[i].tl().y+rect[i].br().y);
            center_point_pub_.publish(msgs);
        }
      cv::imshow(OPENCV_WINDOW,drawmap);
      cv::imshow(OPENCV_WINDOW_1,binary);
      cv::waitKey(1); 
      }
};//类结尾
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
