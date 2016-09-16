#include "ros/ros.h"
#include <string>
#include <iostream>
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include "gazebo_msgs/GetPhysicsProperties.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "frontier.h"
#include <keyboard/Key.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
std::string modelName = "turtlebot";

ros::ServiceClient gmscl;
ros::ServiceClient gphspro;
ros::ServiceClient client;
int carpan = -1;
int counter = 0.2;

// publisher handles
ros::Publisher goal_pub;
ros::Publisher distancePub;
ros::Publisher anglePub;
geometry_msgs::PoseStamped gazeboRobotPose;
frontier *front;
void keyboardCallback(const keyboard::KeyConstPtr& keyconsptr) {
	int key = keyconsptr->code;
	ROS_INFO("key = %d ", key);

	gazebo_msgs::GetModelState getmodelstate;
	getmodelstate.request.model_name = modelName;
	gmscl.call(getmodelstate);

	gazebo_msgs::GetPhysicsProperties getphysicsproperties;
	gphspro.call(getphysicsproperties);

	geometry_msgs::Pose pose;

	geometry_msgs::Twist twist;

	if (key == 273) {
		twist.linear.x = getmodelstate.response.twist.linear.x+0.15;
		twist.angular.z=getmodelstate.response.twist.angular.z+0.15;
		pose.position.x = getmodelstate.response.pose.position.x + 0.15;

	} else if (key == 274) {
		twist.linear.x = getmodelstate.response.twist.linear.x-0.15;;
		twist.angular.z=getmodelstate.response.twist.angular.z-0.15;
		pose.position.x = getmodelstate.response.pose.position.x - 0.15;
	} else {
		pose.position.x = getmodelstate.response.pose.position.x;
	}


	if (key == 275) {
		twist.linear.x = getmodelstate.response.twist.linear.x+0.05;
				twist.angular.z=getmodelstate.response.twist.angular.z+0.05;
		pose.position.y = getmodelstate.response.pose.position.y + 0.05;
	} else if (key == 276) {
		pose.position.y = getmodelstate.response.pose.position.y - 0.05;
		twist.linear.x = getmodelstate.response.twist.linear.x-0.05;;
				twist.angular.z=getmodelstate.response.twist.angular.z-0.05;
	} else {
		pose.position.y = getmodelstate.response.pose.position.y;
	}

	if (counter % 2 == 0) {
		carpan = carpan * 1;
	} else {
		carpan = carpan * -1;
	}

	/**/

	pose.position.z = 0.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;

	gazebo_msgs::SetModelState setmodelstate;
	gazebo_msgs::ModelState modelstate;
	modelstate.model_name = modelName;
	modelstate.pose = pose;
	modelstate.twist = twist;

	setmodelstate.request.model_state = modelstate;

	if (client.call(setmodelstate)) {
		ROS_INFO("BRILLIANT AGAIN!!!");
		ROS_INFO("getphysicsproperties.response.time_step : %f", getphysicsproperties.response.time_step);
		ROS_INFO("modelstate.pose.position.x              :%f", modelstate.pose.position.x);
		ROS_INFO("modelstate.pose.position.y              :%f", modelstate.pose.position.y);
		ROS_INFO("modelstate.twist.linear.x               :%f", modelstate.twist.linear.x);

		ROS_INFO("getmodelstate.response.pose.position.y  :%f", getmodelstate.response.pose.position.y);

		ROS_INFO("getmodelstate.response.orientation.orientation.x  :%f", getmodelstate.response.pose.orientation.x);
		ROS_INFO("getmodelstate.response.orientation.orientation.y  :%f", getmodelstate.response.pose.orientation.y);

		//robot position publish

		gazeboRobotPose.header.frame_id = "/map_cu";
		gazeboRobotPose.pose.position.x = modelstate.pose.position.x;
		gazeboRobotPose.pose.position.y = modelstate.pose.position.y;
		gazeboRobotPose.pose.position.z = modelstate.pose.position.z;
		gazeboRobotPose.pose.orientation.w = getmodelstate.response.pose.orientation.w;
		gazeboRobotPose.pose.orientation.x = getmodelstate.response.pose.orientation.x;
		gazeboRobotPose.pose.orientation.y = getmodelstate.response.pose.orientation.y;
		gazeboRobotPose.pose.orientation.z = getmodelstate.response.pose.orientation.z;

	    goal_pub.publish(gazeboRobotPose);
	} else {
		ROS_ERROR("Failed to call service for setmodelstate ");

	}

}


void currentPosCallback(const geometry_msgs::PoseStampedConstPtr &slamPose)
{
 double xPos;
 double yPos;
 double zPos;
 xPos = slamPose->pose.position.x+2;    //başlangıç noktası 2,2 başladığı için
 yPos = slamPose->pose.position.y+2;

 geometry_msgs::PoseStamped slamRobotPose;

 slamRobotPose.header.frame_id = "/map_cu";
 slamRobotPose.pose.position.x = xPos;
 slamRobotPose.pose.position.y = yPos;
 slamRobotPose.pose.position.z = slamPose->pose.position.z;
 slamRobotPose.pose.orientation.w = slamPose->pose.orientation.w;
 slamRobotPose.pose.orientation.x = slamPose->pose.orientation.x;
 slamRobotPose.pose.orientation.y = slamPose->pose.orientation.y;
 slamRobotPose.pose.orientation.z = slamPose->pose.orientation.z;


 float angleDiff=front->angleDifference(slamRobotPose,gazeboRobotPose);


 std_msgs::Float64  angle;
 angle.data = angleDiff;
 anglePub.publish(angle);

 ROS_INFO("tangleDiff :%lf \n ",angleDiff);

 float farkx=slamRobotPose.pose.position.x-gazeboRobotPose.pose.position.x;
 float farky=slamRobotPose.pose.position.y-gazeboRobotPose.pose.position.y;

 float distanceDiff=sqrt(pow(farkx,2)+pow(farky,2));
 ROS_INFO("distanceDiff :%lf \n ",distanceDiff);

 std_msgs::Float64 distance;
 distance.data = distanceDiff;
 distancePub.publish(distance);


}

int main(int argc, char** argv) {

	ros::init(argc, argv, "turtlebot");
	front=new frontier();
	ros::NodeHandle n;
	ros::Subscriber subKey  = n.subscribe("/keyboard/keydown", 10, keyboardCallback);

	ros::Subscriber slamPse = n.subscribe("/slam_out_pose", 10, currentPosCallback);

	// set up publisher
	goal_pub    = n.advertise<geometry_msgs::PoseStamped>("/gazeboModelPose", 10);
	ros::Publisher keydownPublisher=n.advertise<keyboard::Key>("/keyboard/keydown", 10);;
	distancePub = n.advertise<std_msgs::Float64>("/distanceDiff", 10);
	anglePub    = n.advertise<std_msgs::Float64>("/angleDiff", 10);

	gmscl = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	gphspro = n.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
	client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	gmscl.waitForExistence();
	gphspro.waitForExistence();
	client.waitForExistence();


	//ros::Rate r(11);
	/*keyboard::Key keyboard;
	int min=274;
	int max=275;
		while (ros::ok()) {
			int key =min + (rand() % (int)(max - min + 1));
			//int key=rand() % 276 + 273;

			keyboard.code=key;
            keyboard.header.stamp=ros::Time::now();
			keydownPublisher.publish(keyboard);

			ros::spinOnce();
			r.sleep();
		}*/
	ros::spin();
	return 1;
}
