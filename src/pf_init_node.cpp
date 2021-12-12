#include <ros/ros.h>
#include "pf_init.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pf_initializer");

	PFInitializer pfInit("/davinci_endo/left/image_raw", "/davinci_endo/right/image_raw", false);
	ros::spin();

	return 0;
}