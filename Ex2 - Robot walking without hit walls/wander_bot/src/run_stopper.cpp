/*
 * run_stopper.cpp
 *
 *  Created on: Oct 27, 2016
 *      Author: viki
 */

#include "Stopper.h"

int main(int argc, char **argv) {
	// Initiate new ROS node named "stopper"
	ros::init(argc, argv, "stopper");
	ros::NodeHandle nh("~");
	double forward_speed;
	if (nh.hasParam("forward_speed"))
	{
		nh.getParam("forward_speed", forward_speed);
	}
	else{
		forward_speed = 0.5;
	}
	// Create new stopper object
	Stopper stopper(forward_speed);

	// Start the movement
	stopper.startMoving();

	return 0;
};



