#ifndef PF_INIT_NODE_INCLUDE_PF_INIT_NODE_TEMPLATE_MATCH_H_
#define PF_INIT_NODE_INCLUDE_PF_INIT_NODE_TEMPLATE_MATCH_H_

#include <opencv2/imgproc.hpp>
#include <utility>

/**
 * @brief Represents a match of a needle template on an image
 */
class TemplateMatch {
 public:
	// The z depth, yaw, pitch, roll values used to
	// generate the template in this match
	double z = 0;
	double yaw = 0;
	double pitch = 0;
	double roll = 0;

	// Match quality value returned by cv::matchTemplate()
	double score = -DBL_MAX;

	// Location of origin of template in the match
	cv::Point2i origin;
	// Bounding box of the template
	cv::Rect rect;

	/**
	 * @brief Constructor
	 *
	 * @param score  Match quality value given to match by cv::matchTemplate()
	 * @param rect   Rectangle representing bounds of template in matched image
	 * @param result Result image returned by cv::matchTemplate()
	 * @param templ  Template used in match
	 */
	TemplateMatch(double score, const cv::Rect2i &rect) : score(score), rect(rect) {}

	/**
	 * @brief Default constructor (angle=0, score=-DBL_MAX, rect=(0,0,0,0))
	 */
	TemplateMatch() = default;
};

/**
 * @brief Compares two template matches by score. Orders result s.t. a sorting
 * operation with this comparator will yield smallest values first
 */
class TemplateMatchComparator {
 public:
	int operator()(const TemplateMatch &t1, const TemplateMatch &t2) const {
		return t1.score > t2.score;
	}
};

#endif

