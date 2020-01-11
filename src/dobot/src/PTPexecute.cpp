#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include <dobot/PTPAction.h>
#include "dobot/SetPTPCmd.h"
#include "dobot/GetPose.h"
#include "DobotDll.h"
#include<cmath>
typedef actionlib::SimpleActionServer<dobot::PTPAction> Server;
using namespace std;
class PTPexecute
{
public:
    PTPexecute(std::string name): action_name(name),
    PTPserver(n, name, boost::bind(&PTPexecute::executeCB, this, _1), false),
    client_pose(n.serviceClient<dobot::GetPose>("/DobotServer/GetPose")),
    client_PTP(n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd"))
    {
        PTPserver.start(); 
        cout<<"服务器开启！"<<endl;
    }
    ~PTPexecute(){}
    bool error(const dobot::PTPGoal& goal, const dobot::GetPose& srv)
        {
            double error = fabs(goal.axis[0]-srv.response.x)+fabs(goal.axis[1]-srv.response.y);
            
            if(error > 10)
               return true;
                else
                  return false;
        }
    bool Alarm(dobot::SetPTPCmd::Request req)
    {
            if(sqrt(pow(req.x,2)+pow(req.y,2))>297 || sqrt(pow(req.x,2)+pow(req.y,2))<193)
            return true;

    }
    void executeCB(const dobot::PTPGoalConstPtr& goal)    
        {
            ROS_INFO("executeCb!");         
            while(1)
            {
                    cout<<endl<<endl<<endl;
                    cout<<"-----------------------------"<<endl;
                    this->client_pose.call(this->srv_pose);
                    cout<<"目标值："<<goal->axis[0]<<" 当前值："<<srv_pose.response.x<<endl;
                    cout<<"目标值："<<goal->axis[1]<<" 当前值："<<srv_pose.response.y<<endl;
                    cout<<"-----------------------------"<<endl;
                    if(error(*goal,srv_pose))
                        {
                            this->client_PTP = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
                            this->srv_PTP.request.ptpMode =1;
                            this->srv_PTP.request.x = goal->axis[0];
                            this->srv_PTP.request.y = goal->axis[1];
                            this->srv_PTP.request.z = -80;
                            if(Alarm(srv_PTP.request))
                                {
                                   ROS_INFO("Limited Alarmed！"); 
                                   break;
                                }
                            this->client_PTP.call(this->srv_PTP);    
                            cout<<"--->"<<endl;  
                           
                        }
                    else
                    cout<<"未移动"<<endl;
                       break;
                }
            //goal->active=0;
            dobot::PTPFeedback feedback;
            feedback.info = "收到反馈"; 
            this->PTPserver.publishFeedback(feedback);
            this->PTPserver.setSucceeded();
        }
    
protected:
    ros::NodeHandle n;
    ros::ServiceClient client_PTP;
    ros::ServiceClient client_pose;    
    Server PTPserver;
    dobot::SetPTPCmd srv_PTP;
    dobot::GetPose srv_pose;
    std::string action_name;
    dobot::PTPFeedback feedback;
    dobot::PTPResult result;
};
int main(int argc, char**argv)
{
    ros::init(argc,argv,"PTPexecute");
    PTPexecute a("PTP");
    ros::spin();
    return 0;
}