#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetPhysicsProperties.h"

int main(int argc, char** argv)

{

    ros::init(argc, argv, "kouna_to_asistola");

    ros::NodeHandle n;


    ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::ModelState modelstate;
    modelstate.model_name ="turtlebot";

    setmodelstate.request.model_state=modelstate;

while(ros::ok())
if (client.call(setmodelstate))
      {
       ROS_INFO("BRILLIANT AGAIN!!!");
       ROS_INFO("%f",modelstate.pose.position.x);
       ROS_INFO("%f",modelstate.pose.position.y);
       ROS_INFO("%f",modelstate.twist.linear.x);
      }
     else
      {
       ROS_ERROR("Failed to call service for setmodelstate ");
       return 1;
      }




}
