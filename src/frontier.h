/*
 * frontier.h
 *
 *  Created on: 4 Eyl 2015
 *      Author: nuri
 */

#ifndef SRC_FRONTIER_H_
#define SRC_FRONTIER_H_
#include <boost/shared_array.hpp>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/OccupancyGrid.h>
 #include <tf/transform_listener.h>
class frontier {
public:
	const unsigned char* occupancy_grid_array_;
	boost::shared_array<int> frontier_map_array_;
	unsigned int map_width_;
	unsigned int map_height_;
	unsigned int num_map_cells_;
	int NO_INFORMATION;
	int INSCRIBED_INFLATED_OBSTACLE;
	frontier();
	frontier(int map_width,int map_height);
	void resetMaps();
	void clearFrontiers();
	bool isValid(int point);
	bool isFree(int point);
	bool isFreeFrontiers(int point);
	bool isFrontier(int point);
	float angleDifference(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal);
	double getYawToUnknown(int point);
	bool isFrontierReached(int point);
	bool isSameFrontier(int frontier_point1, int frontier_point2);
	void getAdjacentPoints(int point, int points[]);
	int left(int point);
	int upleft(int point);
	int up(int point);
	int upright(int point);
	int right(int point);
	int downright(int point);
	int down(int point);
	int downleft(int point);
};

#endif /* SRC_FRONTIER_H_ */
