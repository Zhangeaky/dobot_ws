#include<action/DoDishesAction.h>
#include<actionlib/server/simple_action_server.h>
#include<ros/ros.h>
typedef actionlib::SimpleActionServer<action::DoDishesAction> Server;
void execute(const action::DoDishesGoalConstPtr& goal, Server* as);
int main(int argc, char** argv)
{
    ros::init(argc,argv,"server");
    ros::NodeHandle n;
    Server server(n,"do_dishes",boost::bind(&execute, _1,&server),false);
    server.start();
    ros::spin();
    return 0;
}
void execute(const action::DoDishesGoalConstPtr& goal, Server* as)
{
    ros::Rate r(1);
    action::DoDishesFeedback feedback;
    ROS_INFO("Dishwasher %d is working .", goal->dishwasher_id);
    for(int i=1;i<=10;i++)
    {
        feedback.percent_compelete = i*10;
        as->publishFeedback(feedback);
        std::cout<<i<<std::endl;
        r.sleep();
    }
    ROS_INFO("Dishwasher %d finish working.",goal->dishwasher_id);
    as->setSucceeded();
}
