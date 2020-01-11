#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <opencv/cv.h>
using namespace std;
int main(int argc, char** argv)
{
  ros::init(argc,argv,"shot_video");
  ros::NodeHandle n;

	cv::VideoCapture cam(1);
	if (!cam.isOpened())
	{
		cout << "cam open failed!" << endl;
		getchar();
		return -1;
	}

	cout << "cam open success!" << endl;
	//cv::namedWindow("cam");
	cv::Mat img;
	cv::VideoWriter vw;
	int fps = cam.get(cv::CAP_PROP_FPS);  //获取摄像机帧率
	if (fps <= 0)fps = 25;
	//创建视频文件
	vw.open("out.avi", //路径
		cv::VideoWriter::fourcc('X', '2', '6', '4'), //编码格式
		fps, //帧率
		cv::Size(cam.get(cv::CAP_PROP_FRAME_WIDTH),
			cam.get(cv::CAP_PROP_FRAME_HEIGHT))  //尺寸
		);
	if (!vw.isOpened())
	{
		cout << "VideoWriter open failed!" << endl;
		getchar();
		return -1;
	}
	cout << "VideoWriter open success!" << endl;
 
	for (;;)
	{
		cam.read(img);
		if (img.empty())break;
		imshow("cam", img);
		//写入视频文件
		vw.write(img);
		if (cv::waitKey(5) == 'q') break;
	}
 
	cv::waitKey(0);
	return 0;
}
