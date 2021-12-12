#ifndef PF_INIT_NODE_INCLUDE_PF_INIT_NODE_PF_INIT_CONSTANTS_H_
#define PF_INIT_NODE_INCLUDE_PF_INIT_NODE_PF_INIT_CONSTANTS_H_

#include <opencv2/core.hpp>

// Namespace containing constants for particle filter initialization
namespace pfc {
	//Degree 2 radians conversion constant
	const double deg2rad = M_PI/180.0;
	const double rad2deg = 180.0/M_PI;

	//HSV Filtering Parameters
	const int low_h = 0;
	const int high_h = 360;
	const int low_s = 0;
	const int high_s = 0;
	const int low_v = 0;
	const int high_v = 140;

	//Match drawing constants
	const int line_weight = 1;
	const int line_type = 8;
	const int shift = 0;

	// Template Generation, needle radius
	const double radius = 0.0128;

	// Kernel size for gaussian blur of images
	const auto kernel_size = cv::Size(3, 3);

	// Closeness threshold for similar match suppression
	const double match_dist_thresh = 0.8;

	// Camera specific parameters (could be collected from CameraInfo)
	const int endo_x_res = 640;
	const int endo_y_res = 480;

	// Template Match Rotation Parameters
	struct matchConfig {
	  int min_yaw;
	  int max_yaw;
	  int yaw_inc;
	  int min_pitch;
	  int max_pitch;
	  int pitch_inc;
	  int min_roll;
	  int max_roll;
	  int roll_inc;

	  double min_z;
	  double max_z; // Inclusive
	  double z_inc;

	  int num_cand_pts; // The top n matches to keep as candidate points
	  int resolution; // # points in needle line
	};

	// Configure Default Match Parameters
	const matchConfig cfg = {
		0, 360, 30, //yaw
		0, 360, 30, //pitch
		0, 360, 30, //roll
		0.07, 0.18, 0.01, //z
		30, // # candidate points to return
		10, // # points in needle line
	};
}

#endif //PF_INIT_NODE_INCLUDE_PF_INIT_NODE_PF_INIT_CONSTANTS_H_
