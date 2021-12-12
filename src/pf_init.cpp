#include "util.h"
#include "pf_init.h"
#include "matching.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_datatypes.h>

PFInitializer::PFInitializer(const std::string &left_endo_topic, const std::string &right_endo_topic, bool multi_thread): multi_thread_(multi_thread) {
	// Endoscope image subscribers
	endo_sub_l_ = nh_.subscribe(left_endo_topic, 1, &PFInitializer::leftEndoCallback, this);
	endo_sub_r_ = nh_.subscribe(right_endo_topic, 1, &PFInitializer::rightEndoCallback, this);

	// Get endoscope camera projection matrics
	ROS_INFO("Waiting on endoscope camera intrinsics");
	sensor_msgs::CameraInfo l_inf = *(
		ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/davinci_endo/left/camera_info", nh_)
	);
	sensor_msgs::CameraInfo r_inf = *(
		ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/davinci_endo/right/camera_info", nh_)
	);
	ROS_INFO("Received endoscope camera intrinsics");

	proj_l_ = cv::Mat(3, 4, CV_64FC1, (void *)l_inf.P.data());
	proj_r_ = cv::Mat(3, 4, CV_64FC1, (void *)r_inf.P.data());

	// Advertise the candidate posearray service
	service_ = nh_.advertiseService("pf_initializer/pf_candidates", &PFInitializer::generatePFCandidates_, this);
}

// TODO: Include the images to run on in the service call
void PFInitializer::leftEndoCallback(const sensor_msgs::ImageConstPtr &img) {
	try {
		cv::Mat src = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8)->image;
		img_l = src.clone();
	} catch (cv_bridge::Exception const &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
	}
}

void PFInitializer::rightEndoCallback(const sensor_msgs::ImageConstPtr &img) {
	try {
		cv::Mat src = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8)->image;
		img_r = src.clone();
	} catch (cv_bridge::Exception const &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
	}
}


bool PFInitializer::generatePFCandidates_(pf_init_node::CandPoseArray::Request &req,
										  pf_init_node::CandPoseArray::Response &res) {
	// Confirm there are images to run on
	if (!img_l.empty() && !img_r.empty()) {
		// Filter the image
		cv::Mat seg_l = segmentNeedle(img_l);
		cv::Mat seg_r = segmentNeedle(img_r);

		// Collect the template matches
		std::vector<TemplateMatch> l_matches;
		std::vector<TemplateMatch> r_matches;
		if (multi_thread_) {
			l_matches = matchThreaded(seg_l, proj_l_, pfc::cfg.min_pitch, pfc::cfg.max_pitch, pfc::cfg.pitch_inc);
			r_matches = matchThreaded(seg_l, proj_r_, pfc::cfg.min_pitch, pfc::cfg.max_pitch, pfc::cfg.pitch_inc);
		} else {
			l_matches = match(seg_l, proj_l_, pfc::cfg);
			l_matches = collect_top_matches(l_matches, pfc::cfg.num_cand_pts);
			r_matches = match(seg_l, proj_r_, pfc::cfg);
			r_matches = collect_top_matches(r_matches, pfc::cfg.num_cand_pts);
		}

		geometry_msgs::PoseArray pose_arr;

		// Generate candidate points from matches
		// Compute needle positions for all matches
		for (int i = 0; i < l_matches.size(); i++) {
			TemplateMatch match_l = l_matches.at(i);
			TemplateMatch match_r = r_matches.at(i);

			//Initialize left and right needle pixel location vectors
			cv::Mat p_l(2, 1, CV_64FC1);
			cv::Mat p_r(2, 1, CV_64FC1);

			p_l.at<double>(0) = match_l.origin.x;
			p_l.at<double>(1) = match_l.origin.y;

			p_r.at<double>(0) = match_r.origin.x;
			p_r.at<double>(1) = match_r.origin.y;

			// Get 3D location of needle
			cv::Point3d location = deProjectPoints(p_l, p_r, proj_l_, proj_r_);

			// Average out orientation values between left and right calculation
			double avg_yaw = (match_l.yaw + match_r.yaw) / 2.0;
			double avg_pitch = (match_l.pitch + match_r.pitch) / 2.0;
			double avg_roll = (match_l.roll + match_r.roll) / 2.0;
			Eigen::Quaterniond q = rpy_to_quaternion(avg_roll, avg_pitch, avg_yaw);
			geometry_msgs::Pose pose;
			pose.position.x = location.x;
			pose.position.y = location.y;
			pose.position.z = location.z;
			pose.orientation.x = q.x();
			pose.orientation.y = q.y();
			pose.orientation.z = q.z();
			pose.orientation.w = q.w();

			// Store location/orientation
			pose_arr.poses.push_back(pose);
		}
		pose_arr.header.stamp = ros::Time::now();
		// Computed with respect to the left camera
		pose_arr.header.frame_id = "davinci_endo_cam_l";
		res.poses = pose_arr;
		return true;
	}
	return false;
}