#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include "sensor_msgs/LaserScan.h"

ros::Publisher cmd_vel_publisher;
ros::Publisher velocity_stamp_publisher_;
geometry_msgs::TwistStamped velocity_stamp;

sensor_msgs::LaserScanConstPtr prevScan;

double error = 0.0;
double ref = 1.0;
double linearZ = 0.0;
float twist_linear_x = -1;
void sonarCallback(const sensor_msgs::Range& sonar) {

	double error = (ref - sonar.range);
	double kp = 0.3;

	geometry_msgs::Twist twist;
	if (twist_linear_x > -1)
		twist.linear.x = twist_linear_x;
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
	double sum = 0.0;
	for (int i = 0; i < path->poses.size() - 1; i++) {

		sum += sqrt(
				pow((path->poses[i + 1].pose.position.x - path->poses[i].pose.position.x), 2)
						+ pow((path->poses[i + 1].pose.position.y - path->poses[i].pose.position.y), 2));

	}
	ROS_INFO("time => %f (second)--- total trajectory distance  => %d ",path->header.stamp.sec ,sum);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
	double sum = 0.0;
	if (prevScan != NULL && prevScan->ranges.size() - 1 > 0) {

		for (int i = 0; i < scan_in->ranges.size(); i++) {
			//ROS_INFO("ranges[%d] : %f", i, scan_in->ranges[i]);

			sum += abs(prevScan->ranges[i] - scan_in->ranges[i]);
		}
	}

	ROS_INFO("scan Sum Diff : %f", (sum / scan_in->ranges.size()));
	prevScan = scan_in;
}

void cmdVel(const geometry_msgs::Twist &twist) {
	twist_linear_x = twist.linear.x;
}
void scanMap(const nav_msgs::OccupancyGridConstPtr & map) {

	int sum = 0;
	for (int i = 0; i <map->data.size(); i++) {
		if (map->data[i] == 0) {
			sum++;
		}
	}
	ROS_INFO("time => %f (second)--- percantage of cover cell => %f --- total covered  => %d ",map->header.stamp.sec ,
			((double)sum/map->data.size()),sum);

}
int main(int argc, char **argv) {
	ros::init(argc, argv, "tubitak_sonar_controller");
	ros::NodeHandle n;

	cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

	velocity_stamp_publisher_ = n.advertise<geometry_msgs::TwistStamped>("cmd_stamp_vel", 1);

	//ros::Subscriber subPosition = n.subscribe("/sonar_height", 1, sonarCallback);

	//ros::Subscriber subTwist = n.subscribe("/cmd_vel", 10, cmdVel);

	ros::Subscriber subMap = n.subscribe("/map", 1, scanMap);

	ros::Subscriber subTrajectory = n.subscribe("/trajectory", 1, trajectoryCallback);

	//ros::Subscriber subScan = n.subscribe("/scan", 10, scanCallback);

	ros::Rate r(10);
	while (ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
