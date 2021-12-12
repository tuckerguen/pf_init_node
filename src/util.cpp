#include "util.h"
#include "pf_init_constants.h"
#include <opencv2/sfm/triangulation.hpp>

cv::Point3d deProjectPoints(const cv::Mat &p_l, const cv::Mat &p_r, const cv::Mat &P_l, const cv::Mat &P_r) {
	// Create points vector
	std::vector<cv::Mat> points;
	points.push_back(p_l);
	points.push_back(p_r);

	// Create projection matrices vector
	std::vector<cv::Mat> projections;
	projections.push_back(P_l);
	projections.push_back(P_r);

	cv::Mat results;
	// Compute 3D location given points and projection matrices
	cv::sfm::triangulatePoints(points, projections, results);

	// Format results
	cv::Point3d result;
	result.x = results.at<double>(0);
	result.y = results.at<double>(1);
	result.z = results.at<double>(2);

	return result;
}

cv::Mat segmentNeedle(const cv::Mat &img) {
	cv::Mat seg;
	// convert to HSV
	cv::cvtColor(img, seg, cv::COLOR_BGR2HSV);
	// filter by HSV values
	cv::inRange(seg,
				cv::Scalar(pfc::low_h, pfc::low_s, pfc::low_v),
				cv::Scalar(pfc::high_h, pfc::high_s, pfc::high_v),
				seg);
	return seg;
}

double constrainAngle(double x, bool is_deg) {
	double min = is_deg ? 180 : M_PI;
	double max = is_deg ? 360 : 2*M_PI;
	x = fmod(x + min, max);
	if (x < 0)
		x += max;
	return x - min;
}

cv::Point2d calcUVPoint(const cv::Mat &point, const cv::Mat &projection) {
	//Transform into u, v
	cv::Mat needle_arc_pt_uv = projection * point;

	// Divide out homogeneous coordinate
	double u = needle_arc_pt_uv.at<double>(0)/needle_arc_pt_uv.at<double>(2);
	double v = needle_arc_pt_uv.at<double>(1)/needle_arc_pt_uv.at<double>(2);

	return cv::Point2i((int)std::round(u), (int)std::round(v));
}

Eigen::Quaterniond rpy_to_quaternion(double r, double p, double y) {
	return Eigen::Quaterniond(
		Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())
			*Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY())
			*Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()));
}

template<typename T, typename C>
std::vector<T> pq_to_vector(std::priority_queue<T, std::vector<T>, C> pq) {
	std::vector<T> vec;
	while (!pq.empty()) {
		T elem = pq.top();
		vec.push_back(elem);
		pq.pop();
	}
	return vec;
}