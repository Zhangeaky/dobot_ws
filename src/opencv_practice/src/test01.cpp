#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgcodecs/imgcodecs.hpp>
#include<iostream>
#include<ros/ros.h>
using namespace std;

int main()
{
    //cv::Mat src = cv::imread();
    cv::namedWindow("video_record", cv::WINDOW_AUTOSIZE);
    cv::VideoCapture cap(0);
    cv::Mat img;
    cv::VideoWriter record;

    int fps =cap.get(cv::CAP_PROP_FPS);
    if(fps <= 0)
    {
        fps = 25;
    }
    record.open("out.avi", cv::VideoWriter::fourcc('x', '2','6','4'), fps,
    cv::Size(cap.get(cv::CAP_PROP_FRAME_HEIGHT),cap.get(cv::CAP_PROP_FRAME_WIDTH))
    );
    if(!cap.isOpened())
        {
            ROS_ERROR("failed \n");
        }
    if(!record.isOpened())
        {
            ROS_ERROR("record failed \n");
        }
    
    while(1)
        {
            cap.read(img);
            if(img.empty())
                break;
            imshow("video_record", img);
            record.write(img);

            if(cv::waitKey(10 == 'q'))
                break;

        }
}