#ifndef PF_INIT_NODE_INCLUDE_PF_INIT_NODE_UTIL_H_
#define PF_INIT_NODE_INCLUDE_PF_INIT_NODE_UTIL_H_

#include <queue>
#include "template_match.h"
#include <opencv2/core.hpp>
#include <eigen3/Eigen/Dense>

/**
 * @brief Segment needle from endoscope image
 * @param img image to segment
 * @return segmented image (binary mask)
 */
cv::Mat segmentNeedle(const cv::Mat& img);

/**
 * @brief Constrain angle x to stay between PI and 2*PI
 * @param x angle to constrain
 * @param is_deg true if x is in degrees, false if radians
 * @return the constrained angle between PI and 2*Pi
 */
double constrainAngle(double x, bool is_deg);

/**
 * @brief Compute the image point from a given 3D point by projection
 * @param point The point to compute the image point for
 * @param projection the projection matrix
 * @return The (u, v) coordinate of the projected point
 */
cv::Point2d calcUVPoint(const cv::Mat &point, const cv::Mat &projection);

template<typename T, typename C>
/**
 * @brief Converts a priority queue to a vector in order
 * @tparam T element type
 * @tparam C comparator type for comparing elements of the priority queue
 * @param pq priority queue to convert
 * @return vector of priority queue elements ordered from greatest to least
 */
std::vector<T> pq_to_vector(std::priority_queue<T, std::vector<T>, C> pq);

 /**
  * @brief returns 3D location of point given two pixel correspondences and the intrinsic camera
  * matrices of the cameras the images were taken with
  * @param p_l points in the left image
  * @param p_r points in the right image
  * @param P_l left camera projection matrix
  * @param P_r right camera projection matrix
  * @return 3D point triangulated from the corresponding points
  */
cv::Point3d deProjectPoints(const cv::Mat& p_l, const cv::Mat& p_r, const cv::Mat& P_l, const cv::Mat& P_r);

/**
 * @brief Convert euler angles, (roll, pitch, yaw), to quaternion
 * @param r roll
 * @param p pitch
 * @param y yaw
 * @return quaternion equivalent
 */
Eigen::Quaterniond rpy_to_quaternion(double r, double p, double y);

#endif //PF_INIT_NODE_INCLUDE_PF_INIT_NODE_UTIL_H_
