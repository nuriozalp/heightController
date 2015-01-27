#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <sstream>

ros::Publisher cmd_vel_publisher;
ros::Publisher velocity_stamp_publisher_;
geometry_msgs::TwistStamped velocity_stamp;
double error = 0.0;
double ref = 1.0;
double linearZ = 0.0;
void sonarCallback(const sensor_msgs::Range& sonar) {

	double error = (ref - sonar.range);
	double kp = 0.3;

	geometry_msgs::Twist twist;
	twist.linear.z = kp * error;
	linearZ = kp * error;
	ROS_INFO("kp*error : %f :", kp * error);

	cmd_vel_publisher.publish(twist);

	velocity_stamp.header.stamp = ros::Time::now();
	velocity_stamp.twist.linear.x = twist.linear.x;
	velocity_stamp.twist.angular.z = twist.angular.z;
	velocity_stamp_publisher_.publish(velocity_stamp);

}

void trajectoryCallback(const nav_msgs::PathConstPtr& path) {
	double sum=0.0;
	for (int i = 0; i < path->poses.size()-1; i++) {

		sum+=sqrt(pow((path->poses[i+1].pose.position.x-path->poses[i].pose.position.x),2)+
				pow((path->poses[i+1].pose.position.y-path->poses[i].pose.position.y),2));


	}
	ROS_INFO("sum : %f", sum);

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "tubitak_sonar_controller");
	ros::NodeHandle n;

	cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	velocity_stamp_publisher_ = n.advertise<geometry_msgs::TwistStamped>("cmd_stamp_vel", 10);

	//ros::Subscriber subPosition = n.subscribe("/sonar_height", 1, sonarCallback);

	ros::Subscriber subTrajectory = n.subscribe("/trajectory", 1, trajectoryCallback);

	ros::Rate r(10);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
