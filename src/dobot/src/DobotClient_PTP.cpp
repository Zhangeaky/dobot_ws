/****************************************LIBRARY**********************************************/
#include "ros/ros.h"
#include "axis_tf/getPoint.h"
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
/****************************************DOBOTLIBRARY******************************************/
#include<actionlib/client/simple_action_client.h>
#include<dobot/PTPAction.h>
typedef actionlib::SimpleActionClient<dobot::PTPAction> Client;
/****************************************DOBOTLIBRARY******************************************/
#include "dobot/SetCmdTimeout.h"
#include "dobot/SetQueuedCmdClear.h"
#include "dobot/SetQueuedCmdStartExec.h"
#include "dobot/SetQueuedCmdForceStopExec.h"
#include "dobot/GetDeviceVersion.h"
#include "dobot/SetWAITCmd.h"
#include "dobot/SetEndEffectorParams.h"
#include "dobot/SetPTPJointParams.h"
#include "dobot/SetPTPCoordinateParams.h"
#include "dobot/SetPTPJumpParams.h"
#include "dobot/SetPTPCommonParams.h"
#include "dobot/SetPTPCmd.h"
#include "dobot/SetHOMEParams.h"
#include "dobot/GetHOMEParams.h"
#include "dobot/SetHOMECmd.h"
#include "DobotDll.h"
#include "DobotType.h"
using namespace std;
vector<geometry_msgs::PointStamped> point_queue;
bool flag = 0;
int count_callback = 0;
/************************************dobot变量****************************************************/
dobot::SetPTPCmd* pointer_srv_setPTPCmd;
dobot::SetWAITCmd* pointer_srv_setWAITCmd;
/************************************ros变量******************************************************/
ros::ServiceClient* pointer_setPTPCmd;
ros::ServiceClient* pointer_setHomeCmd;
ros::ServiceClient* pointer_setWAITCmd;
ros::NodeHandle* pointer_n;
/*************************************action变量**************************************************/
dobot::PTPGoal goal;
dobot::PTPResult result;
Client* pointer_action_client;

/************************************tf变量*******************************************************/
geometry_msgs::PointStamped camera;
geometry_msgs::PointStamped world;
geometry_msgs::PointStamped base;
tf::TransformListener* pointer_listener;
tf::StampedTransform* pointer_transform_storage;
tf::StampedTransform* pointer_transform_storage_;
void processWorldPoint(geometry_msgs::PointStamped& world);
void dobotGo(geometry_msgs::PointStamped& base);


void callbackGetPoint(const axis_tf::getPointConstPtr msg)
    {
        camera.point.x= msg->x1;
        camera.point.y= msg->x2;
        camera.point.z= msg->x3;
        
        try
            {
                //pointer_listener->waitForTransform("world" ,"logitech",ros::Time(0), ros::Duration(3.0));                
                //pointer_listener->lookupTransform("world","logitech", ros::Time(0), *pointer_transform_storage);//获得想要转换坐标系之间的转换矩阵
                pointer_listener->waitForTransform("dobot_base","logitech",ros::Time(0), ros::Duration(3.0));
                pointer_listener->lookupTransform("world","logitech", ros::Time(0), *pointer_transform_storage);
                pointer_listener->lookupTransform("dobot_base","world", ros::Time(0), *pointer_transform_storage_);
                
                //pointer_listener->transformPose("logitech", "world",global_pose);
                //cout<<"debug!"<<endl;
                //cout<<global_pose<<endl;
            
            }
        catch (tf::TransformException &ex) 
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

        try
            {
                pointer_listener->transformPoint("world",camera, world);
                processWorldPoint(world);
                pointer_listener->transformPoint("dobot_base",world, base);
            }
        catch (tf::TransformException &ex) 
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        dobotGo(base);
        // cout<<endl;
        // cout<<"********************"<<endl;
        // cout<<"x: "<<world.point.x<<endl;
        // cout<<"y: "<<world.point.y<<endl;
        // cout<<"z: "<<world.point.z<<endl;
        // cout<<"********************"<<endl;
        // cout<<endl;
        //processWorldPoint(world);
         
    }
int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;
    pointer_n = &n;
    ros::ServiceClient client;
    /********************************************tf初始化*****************************************/ 
    camera.header.stamp=ros::Time();
    camera.header.frame_id="logitech";
    world.header.stamp = ros::Time();
    world.header.frame_id = "world";
    base.header.stamp = ros::Time();
    base.header.frame_id = "dobot_base";

    ros::Subscriber sub = n.subscribe("result_1", 1000, callbackGetPoint);
    tf::TransformListener listener;
    tf::StampedTransform transform_storage;
    tf::StampedTransform transform_storage_;
    pointer_listener = &listener;
    pointer_transform_storage = &transform_storage;
    pointer_transform_storage_=&transform_storage_;
    /*******************************************action初始化**************************************/ 
    Client client_action("PTP",true);
    pointer_action_client = &client_action;
    goal.active = 0;
    /********************************************机器人初始化**************************************/   
    client = n.serviceClient<dobot::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }
    client = n.serviceClient<dobot::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot::SetQueuedCmdClear srv2;
    client.call(srv2);
    client = n.serviceClient<dobot::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    dobot::SetQueuedCmdStartExec srv3;
    client.call(srv3);
    client = n.serviceClient<dobot::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot::SetEndEffectorParams srv4;
    srv4.request.xBias = 70;//参数被写在srv的请求域，由服务端中回调函数进行赋值
    srv4.request.yBias = 0;
    srv4.request.zBias = 0;
    client.call(srv4);
/**********************************************设置参数*******************************************/
    do {
        client = n.serviceClient<dobot::SetPTPJointParams>("/DobotServer/SetPTPJointParams");
        dobot::SetPTPJointParams srv5;

        for (int i = 0; i < 4; i++) {
            srv5.request.velocity.push_back(100);
        }
        for (int i = 0; i < 4; i++) {
            srv5.request.acceleration.push_back(100);
        }
        client.call(srv5);
    } while (0);
    do {
        client = n.serviceClient<dobot::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        dobot::SetPTPCoordinateParams srv6;
        srv6.request.xyzVelocity = 150;
        srv6.request.xyzAcceleration = 100;
        srv6.request.rVelocity = 100;
        srv6.request.rAcceleration = 100;
        client.call(srv6);
    } while (0);
    do {
        client = n.serviceClient<dobot::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        dobot::SetPTPJumpParams srv7;
        srv7.request.jumpHeight = 20;
        srv7.request.zLimit = 200;
        client.call(srv7);
    } while (0);
    do {
        client = n.serviceClient<dobot::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        dobot::SetPTPCommonParams srv8;

        srv8.request.velocityRatio = 50;
        srv8.request.accelerationRatio = 50;
        srv8.request.isQueued = 0;
        client.call(srv8);
    } while (0);
    
    ros::ServiceClient client_setPTPCmd = n.serviceClient<dobot::SetPTPCmd>("/DobotServer/SetPTPCmd");
    pointer_setPTPCmd = &client_setPTPCmd;
    dobot::SetPTPCmd srv_setPTPCmd;
    pointer_srv_setPTPCmd = &srv_setPTPCmd;

    ros::ServiceClient client_setWAITCmd = n.serviceClient<dobot::SetWAITCmd>("/DobotServer/SetWAITCmd");
    dobot::SetWAITCmd srv_setWAITCmd;
    pointer_setWAITCmd = &client_setWAITCmd;
    pointer_srv_setWAITCmd = &srv_setWAITCmd;
    ros::ServiceClient client_setHomeCmd = n.serviceClient<dobot::SetHOMECmd>("/DobotServer/SetHOMECmd");
    dobot::SetHOMECmd srv_home;
    client_setHomeCmd.call(srv_home);   
    ros::spin();
    return 0;
}
void processWorldPoint(geometry_msgs::PointStamped& world)
    {
        world.point.x = (world.point.x+125)/1000;
        world.point.y = (world.point.y+125)/1000;
        // cout<<"world_x: "<< world.point.x*1000<<endl;
        // cout<<"world_y: "<< world.point.y*1000<<endl;
    }

void comparePoint(vector<geometry_msgs::PointStamped>& vector,bool& flag)
{
    if(vector.size()<3)
        return;
    double error_x = fabs(vector[vector.size()-1].point.x-vector[vector.size()-2].point.x);
    double error_y = fabs(vector[vector.size()-1].point.y-vector[vector.size()-2].point.y);
    if(error_x<0.005&&error_y<0.005)
        flag = true;
        else
        {
           flag = false;
        }
}

bool error(geometry_msgs::PointStamped& base,dobot::PTPGoal goal)
{
    if( count_callback == 1)
        return true;
    if( fabs(base.point.x*1000-goal.axis[0])+fabs(base.point.y*1000-goal.axis[1])>10)
        return true;
        else
        {
            return false;
        }
}

bool Alarm(dobot::SetPTPCmd::Request req)
{
        if(sqrt(pow(req.x,2)+pow(req.y,2))>323 || sqrt(pow(req.x,2)+pow(req.y,2))<193)
        return true;

}
// void dobotGo(geometry_msgs::PointStamped& base)
//     {
//         ++count_callback;
//         if( !error(base,goal) )
//             return;
//         cout<<endl;
//         cout<<"------------------"<<endl;
//         ROS_INFO("count_callback: %d", count_callback);
//         cout<<"x:"<<base.point.x*1000<<endl;   
//         cout<<"y:"<<base.point.y*1000<<endl;
    
//         goal.axis.clear();
//         goal.axis.push_back(base.point.x*1000);
//         goal.axis.push_back(base.point.y*1000);
//         goal.axis.push_back(-80);
//         pointer_srv_setPTPCmd->request.x = base.point.x*1000;
//         pointer_srv_setPTPCmd->request.y = base.point.x*1000;
//         pointer_srv_setPTPCmd->request.z = -80;
//         pointer_srv_setPTPCmd->request.ptpMode = 1;
//         if(Alarm(pointer_srv_setPTPCmd->request))
//             {
//                 ROS_INFO("Limited Alarmed!"); 
//                 return;
//             }
//         cout<<"-------------------------------------"<<endl;
//         pointer_setPTPCmd->call(*pointer_srv_setPTPCmd);  
//     }
   

void activeCb()
{
    ROS_INFO("action is going!");
    
}
void feedbackCb(const dobot::PTPFeedbackConstPtr& feedback)
{   
}
void doneCb(const actionlib::SimpleClientGoalState& state,const dobot::PTPResultConstPtr& result)
{
    goal.active = 0;

    ROS_INFO("job is done!");
}
void dobotGo(geometry_msgs::PointStamped& base)
{
    ++count_callback;
    if( !error(base,goal) )
        return;
    cout<<endl;
    cout<<"------------------"<<endl;
    ROS_INFO("count_callback: %d", count_callback);
    cout<<"x:"<<base.point.x*1000<<endl;   
    cout<<"y:"<<base.point.y*1000<<endl;
   
    goal.axis.clear();
    goal.active = count_callback;
    goal.axis.push_back(base.point.x*1000);
    goal.axis.push_back(base.point.y*1000);
    goal.axis.push_back(-80);
    ROS_INFO("wait to send goal!");
    pointer_action_client->sendGoal(goal,&doneCb,&activeCb,&feedbackCb);  
    cout<<"goal is already sent!"<<endl;
   
    cout<<"------------------"<<endl<<endl;
    //while(goal.active);
}                                                           