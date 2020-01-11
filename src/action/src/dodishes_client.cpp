 #include<action/DoDishesAction.h>
 #include<ros/ros.h>
 #include<actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<action::DoDishesAction> Client;
void doneCb(const actionlib::SimpleClientGoalState& state, const action::DoDishesResultConstPtr& result);
void activeCb();
void feedbackCb(const action::DoDishesFeedbackConstPtr& feedback);
int main(int argc, char** argv)
{
   ros::init(argc,argv,"client"); 
   Client client("do_dishes", true);
   ROS_INFO("Waiting for action server to start");
   client.waitForServer();
   ROS_INFO("action server started, sending goal!");

   action::DoDishesGoal goal;
   goal.dishwasher_id = 0;
   client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

   ros::spin();
   return 0;
} 
void doneCb(const actionlib::SimpleClientGoalState& state, const action::DoDishesResultConstPtr& result)
   {
      ROS_INFO("The dishes are done!");
      ros::shutdown();
   }
void activeCb()
   {
      ROS_INFO("goal is just");
   }
void feedbackCb(const action::DoDishesFeedbackConstPtr& feedback)
   {
      ROS_INFO("percent_complete: %f", feedback->percent_compelete);
   }


   