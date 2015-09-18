/*
 * frontier.cpp
 *
 *  Created on: 4 Eyl 2015
 *      Author: nuri
 */
#include "frontier.h"

frontier::frontier() {
	NO_INFORMATION = -1;
	INSCRIBED_INFLATED_OBSTACLE = 100;
	map_height_=100;
	map_width_=100;
	num_map_cells_=map_height_*map_width_;
}
frontier::frontier(int map_width,int map_height){
	NO_INFORMATION = -1;
		INSCRIBED_INFLATED_OBSTACLE = 100;
		map_height_=map_height;
		map_width_=map_width;
		num_map_cells_=map_height*map_width;
}
void frontier::resetMaps() {

}
void frontier::clearFrontiers(){
  std::fill_n(frontier_map_array_.get(), num_map_cells_, 0);
}
bool frontier::isValid(int point) {
	return (point >= 0);
}
bool frontier::isFree(int point){
	 if(isValid(point)){
	  	      if(occupancy_grid_array_[point] <= INSCRIBED_INFLATED_OBSTACLE){
	        return true;
	      }else if(occupancy_grid_array_[point] == NO_INFORMATION){
	        return true;
	      }
	 }
	  return false;
}
bool frontier::isFreeFrontiers(int point) {
	if (isValid(point)) {
		if (occupancy_grid_array_[point] <= INSCRIBED_INFLATED_OBSTACLE) {
			return true;
		}
	}
	return false;
}
bool frontier::isFrontier(int point) {
	if (isFreeFrontiers(point)) {

		int adjacentPoints[8];
		getAdjacentPoints(point, adjacentPoints);

		for (int i = 0; i < 8; ++i) {
			if (isValid(adjacentPoints[i])) {
				if (occupancy_grid_array_[adjacentPoints[i]] == NO_INFORMATION) {

					int no_inf_count = 0;
					int noInfPoints[8];
					getAdjacentPoints(adjacentPoints[i], noInfPoints);
					for (int j = 0; j < 8; j++) {

						if (isValid(noInfPoints[j]) && occupancy_grid_array_[noInfPoints[j]] == NO_INFORMATION) {
							++no_inf_count;

							if (no_inf_count > 2) {
								return true;
							}
						}
					}
				}
			}
		}
	}

	return false;
}
float frontier::angleDifference(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal) {
	// setup start positions
	unsigned int mxs, mys;
	mxs = start.pose.position.x;
	mys = start.pose.position.y;

	unsigned int gx, gy;
	gx = goal.pose.position.x;
	gy = goal.pose.position.y;

	int goal_proj_x = gx - mxs;
	int goal_proj_y = gy - mys;

	float start_angle = tf::getYaw(start.pose.orientation);
	float goal_angle = std::atan2(goal_proj_y, goal_proj_x);

	float both_angle = 0;
	if (start_angle > goal_angle) {
		both_angle = start_angle - goal_angle;
	} else {
		both_angle = goal_angle - start_angle;
	}

	 if(both_angle > M_PI){
	   both_angle = (M_PI - std::abs(start_angle)) + (M_PI - std::abs(goal_angle));
	 }

	return both_angle;
}
double frontier::getYawToUnknown(int point){
	return 0.0;
}
bool frontier::isFrontierReached(int point){
	return true;
}
bool frontier::isSameFrontier(int frontier_point1, int frontier_point2){
	return true;
}
inline void frontier::getAdjacentPoints(int point, int points[]){

  points[0] = left(point);
  points[1] = up(point);
  points[2] = right(point);
  points[3] = down(point);
  points[4] = upleft(point);
  points[5] = upright(point);
  points[6] = downright(point);
  points[7] = downleft(point);

}
inline int frontier::left(int point) {
	// only go left if no index error and if current point is not already on the left boundary
	if ((point % map_width_ != 0)) {
		return point - 1;
	}
	return -1;
}
inline int frontier::upleft(int point) {
	if ((point % map_width_ != 0) && (point >= (int) map_width_)) {
		return point - 1 - map_width_;
	}
	return -1;

}
inline int frontier::up(int point) {
	if (point >= (int) map_width_) {
		return point - map_width_;
	}
	return -1;
}
inline int frontier::upright(int point) {
	if ((point >= (int) map_width_) && ((point + 1) % (int) map_width_ != 0)) {
		return point - map_width_ + 1;
	}
	return -1;
}
inline int frontier::right(int point) {
	if ((point + 1) % map_width_ != 0) {
		return point + 1;
	}
	return -1;

}
inline int frontier::downright(int point) {
	if (((point + 1) % map_width_ != 0) && ((point / map_width_) < (map_height_ - 1))) {
		return point + map_width_ + 1;
	}
	return -1;

}
inline int frontier::down(int point) {
	if ((point / map_width_) < (map_height_ - 1)) {
		return point + map_width_;
	}
	return -1;

}
inline int frontier::downleft(int point) {
	if (((point / map_width_) < (map_height_ - 1)) && (point % map_width_ != 0)) {
		return point + map_width_ - 1;
	}
	return -1;

}

