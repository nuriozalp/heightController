#include "ros/ros.h"
#include <string>
#include <iostream>
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetPhysicsProperties.h"
std::string modelName="pr2";
int main(int argc, char** argv)
{

  ros::init(argc, argv, "turtlebot");

  ros::NodeHandle n;

  ros::ServiceClient gmscl=n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  ros::ServiceClient gphspro=n.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gmscl.waitForExistence();
  gphspro.waitForExistence();
  client.waitForExistence();

ros::Rate r(1);
int carpan=-1;
int counter=1;
  for (double t=0.1; t<=20000; t+=0.001)
  {

    gazebo_msgs::GetModelState getmodelstate;
    getmodelstate.request.model_name =modelName;
    gmscl.call(getmodelstate);

    gazebo_msgs::GetPhysicsProperties getphysicsproperties;
    gphspro.call(getphysicsproperties);

    geometry_msgs::Pose pose;

    geometry_msgs::Twist twist;

    twist.linear.x = 0.5*carpan;
    twist.linear.y = 0.0*carpan;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0*carpan;
    twist.angular.y = 0.0;
    twist.angular.z = 0.5*carpan;

if(counter%2==0){
carpan=carpan*1;
}else{
carpan=carpan*-1;
}
counter++;
    pose.position.x = getmodelstate.response.pose.position.x + 0.1;
    pose.position.y = getmodelstate.response.pose.position.y + 0.1;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;



    gazebo_msgs::SetModelState setmodelstate;
    gazebo_msgs::ModelState modelstate;
    modelstate.model_name =modelName;
    modelstate.pose = pose;
    modelstate.twist = twist;

    setmodelstate.request.model_state=modelstate;


    if (client.call(setmodelstate))
    {
      ROS_INFO("BRILLIANT AGAIN!!!");
      ROS_INFO("%f",getphysicsproperties.response.time_step);
      ROS_INFO("%f",modelstate.pose.position.x);
      ROS_INFO("%f",modelstate.pose.position.y);
      ROS_INFO("%f",modelstate.twist.linear.x);
      ROS_INFO("%f",getmodelstate.response.pose.position.y);
    }
    else
    {
      ROS_ERROR("Failed to call service for setmodelstate ");
      return 1;
    }

ros::spinOnce();
        r.sleep();
  }
}
