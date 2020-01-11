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
/**********************************HSV***********************************************/
vector<int> hsvred    ={0, 0,10, 50,255, 55,255};
vector<int> hsvyellow ={0, 26,34,   70,255, 75,255};
vector<int> hsvblue   ={0, 100,124, 50,255, 55,255};
vector<int> hsvgreen  ={0, 45,77,   50,255, 55,255};
vector<int> hsvpurple ={0, 125,155, 50,255, 55,255};


static const string OPENCV_WINDOW = "color _distinguish";
static const string OPENCV_WINDOW_1 = "binary";
cv::Mat output_gray;
cv::Mat binary;
cv::Mat mask;

vector<cv::Vec4i> hierarcy;

class cube
{
public:
  cube(const string name,vector<int> hsv)
  {
    this->name = name;
    this->HSV  = hsv;
    
  }
 
  vector<int>& getHSV()
  {
    return this->HSV;
  }
  vector< vector<cv::Point> >&  getContours()
  {
    return this->contours;
  }

private:
  string name;
  vector< vector<cv::Point> >  contours;
  vector<int> HSV;
};
cube red("red",hsvred);
cube yellow("yellow",hsvyellow);
cube blue("blue",hsvblue);
cube green("green",hsvgreen);
cube purple("purple",hsvpurple);


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
    center_point_pub_= nh_.advertise<opencvtest::pixel_point>("pixel_center_axis",1000);//话题名称和接受队列缓存消息条数；
    cv::namedWindow(OPENCV_WINDOW);
}
~ImageConverter()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void process(cv_bridge::CvImagePtr& cv_ptr,cube& color)
{
  cv::Mat drawmap = cv_ptr->image.clone(); //画布
  cv::Mat clone = cv_ptr->image.clone(); 
  
  cv::cvtColor(clone,clone,CV_BGR2HSV);
  cv::inRange(clone,cv::Scalar(color.getHSV()[1],color.getHSV()[3],color.getHSV()[5]),
                 cv::Scalar(color.getHSV()[2],color.getHSV()[4],color.getHSV()[6]),clone);
  binary = clone.clone();
  cv::medianBlur(binary,binary,25);
  
  cv::findContours(clone,color.getContours(), hierarcy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  if(color.getContours().size()==0)
    return;
  cout<<"we detected "<<color.getContours().size()<<"contour(s)"<<endl;
  vector<cv::Rect> rect;
  for(int i = 0;i <color.getContours().size(); i++)
      {
          rect.push_back(cv::boundingRect(color.getContours()[i]));
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
  image_pub_.publish(cv_ptr->toImageMsg());
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
   

  //  process(cv_ptr,red);
      process(cv_ptr,yellow);
  //  process(cv_ptr,blue);
  //  process(cv_ptr,green);
  //  process(cv_ptr,purple);

 
  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_distinguish");
  ImageConverter ic;
  ros::spin();
  return 0;
}
