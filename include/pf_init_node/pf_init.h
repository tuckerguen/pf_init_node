#ifndef PF_INIT_NODE_INCLUDE_PF_INIT_NODE_PF_INIT_H_
#define PF_INIT_NODE_INCLUDE_PF_INIT_NODE_PF_INIT_H_

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include "pf_init_node/CandPoseArray.h"

/**
 * @brief Runs a service that computes a PoseArray of candidate needle positions
 * for the initialization of the needle tracking particle filter
 */
class PFInitializer {
 public:
	/**
	 * @brief Constructor
	 * @param left_endo_topic topic to subscribe to for left endoscope image
	 * @param right_endo_topic topic to subscribe to for right endoscope image
	 * @param multi_thread use multi-threading on template matching or not
	 */
	PFInitializer(const std::string& left_endo_topic, const std::string& right_endo_topic, bool multi_thread);

	/**
	 * @brief Default deconstructor
	 */
	~PFInitializer() = default;

	/**
	 * @brief Callback for the left endoscope image. Converts the image message
	 * to an OpenCV Mat and saves it in this object
	 * @param img image message
	 */
	void leftEndoCallback(const sensor_msgs::ImageConstPtr &img);
	void rightEndoCallback(const sensor_msgs::ImageConstPtr &img);

 private:
	// The node handle to create subscribers and advertise service from
	ros::NodeHandle nh_ = ros::NodeHandle("~");

	// The candidate posearray service
	ros::ServiceServer service_;

	// Endoscope camera subscribers
	ros::Subscriber endo_sub_l_;
	ros::Subscriber endo_sub_r_;

	// Most recent images received
	cv::Mat img_l;
	cv::Mat img_r;

	// Cmaera projection matrices
	cv::Mat proj_l_;
	cv::Mat proj_r_;

	// Use multithreading when doing matching
	bool multi_thread_;

	/**
	 * @brief Service callback for running initialization. Responds with a pose array
	 * of candidate needle poses extracted from the current endoscope images
	 * @param req Service request
	 * @param res Service response
	 * @return whether or not the service call was successful
	 */
	bool generatePFCandidates_(pf_init_node::CandPoseArray::Request &req,
							   pf_init_node::CandPoseArray::Response &res);

};

#endif
