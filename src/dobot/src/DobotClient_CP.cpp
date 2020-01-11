#include "ros/ros.h"
#include "dobot/SetCPParams.h"
#include "dobot/GetCPParams.h"
#include "dobot/SetCPCmd.h"
// /DobotServer/SetCPParams
// /DobotServer/GetCPParams
// /DobotServer/SetCPCmd

class CP
{
public:
    CP(double planAcc=100, double acc = 100, double junctionVel = 100, bool isrealTimeTrack= 1,bool isQueued =0)
    {   ROS_INFO("construction!");
        this->srv_Set.request.planAcc = planAcc;
        this->srv_Set.request.acc =100;
        this->srv_Set.request.realTimeTrack = isrealTimeTrack;
        this->srv_Set.request.junctionVel = junctionVel;
        this->srv_Set.request.isQueued = isQueued;
        client = n.serviceClient<dobot::SetCPParams>("/DobotServer/SetCPParams");
        client.call(this->srv_Set);
        this->move();
    }
    void move()
    {
        ROS_INFO("go");
        this->srv_Cmd.request.cpMode = 0;
        this->srv_Cmd.request.x = 1;
        this->srv_Cmd.request.y = 1;
        this->srv_Cmd.request.z = 0;
        this->srv_Cmd.request.velocity = 100;   
        this->client = n.serviceClient<dobot::SetCPCmd>("/DobotServer/SetCPCmd");
        client.call(this->srv_Cmd);
    }
    ~CP()
    {

    }

private:
    ros::NodeHandle n;
    ros::ServiceClient client;
    dobot::SetCPParams srv_Set;
    dobot::GetCPParams srv_Get;
    dobot::SetCPCmd srv_Cmd;
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "DobotClient_CP");
    CP cp;
    ros::spin();



}