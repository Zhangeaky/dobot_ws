#include "ros/ros.h"
#include "DobotDll.h"
#include "dobot/mypose.h"
#include "dobot/GetPose.h"
#include <actionlib/client/simple_action_client.h>
#include <dobot/poseAction.h>
using namespace std;
class GetPose_
{
public:
   GetPose_():client(n.serviceClient<dobot::GetPose>("/DobotServer/GetPose"))
   {
      ros::Rate r(10);
      while(ros::ok())
      {
         
         client.call(srv);
         cout<<"X轴坐标： "<<srv.response.x<<endl;
         cout<<"Y轴坐标： "<<srv.response.y<<endl;
         cout<<"--------------------------------"<<endl;

         r.sleep();
      }
   }
protected:
   ros::NodeHandle n;
   ros::ServiceClient client;
   dobot::GetPose srv;
};

int main(int argc, char** argv)
{
   ros::init(argc,argv,"publish_pose");
   GetPose_ a;
   ros::spin();
}