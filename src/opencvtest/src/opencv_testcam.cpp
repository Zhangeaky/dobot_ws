/*********************************ROS***********************************************/
#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<image_transport/image_transport.h>
#include<sensor_msgs/image_encodings.h>
/**********************************OPENCVLIBRARIES***********************************/
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>//新式C++风格图像处理函数
#include<opencv2/highgui/highgui.hpp>//C++风格的显示、滑动条鼠标及输入输出相关
#include<opencvtest/pixel_point.h>
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
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
ros::Publisher center_point_pub_;
 
public:
ImageConverter()
: it_(nh_)// 初始化列表//
{
image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,&ImageConverter::imageCb, this);//订阅节点，该节点在USB_cam/image_raw中发布。
//image_pub_ = it_.advertise("/image_converter/output_video", 1);//发布转换后的视频流
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
      try
      {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//转化为opencv格式图像，返回指针
      }
      catch (cv_bridge::Exception& e)
      {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
      }
      cv::Mat drawmap = cv_ptr->image.clone(); //画布，用来画出物块的矩形轮廓。
      //cv::GaussianBlur(cv_ptr->image,cv_ptr->image,11,11,0);
      cv::cvtColor(cv_ptr->image,cv_ptr->image,CV_BGR2HSV);// 转化为灰度图
      cv::inRange(cv_ptr->image,cv::Scalar(hmin,smin,vmin),cv::Scalar(hmax,smax,vmax),cv_ptr->image);
      cv::medianBlur(cv_ptr->image,cv_ptr->image,25);
      binary = cv_ptr->image.clone();
      
      cv::findContours(cv_ptr->image,contours, hierarcy,
                      CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
      cout<<"we detected "<<contours.size()<<"contour(s)"<<endl;
      vector<cv::Rect> rect;
      for(int i = 0;i <contours.size(); i++)
        {
            rect.push_back(cv::boundingRect(contours[i]));
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
      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());
      }
};//类结尾
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
