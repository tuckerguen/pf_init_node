#ifndef PF_INIT_NODE_INCLUDE_PF_INIT_NODE_MATCHING_H_
#define PF_INIT_NODE_INCLUDE_PF_INIT_NODE_MATCHING_H_

#include <vector>
#include "template_match.h"
#include "pf_init_constants.h"


/**
 * @brief Run template matching on a base image with a template generated over range of scales and rotations
 * @param img the image to match the template to
 * @param proj_mat camera projection matrix for template generation
 * @param cfg config specifying the roll, pitch, yaw, z depth to generate templates for
 * @return vector of template matches run
 */
std::vector<TemplateMatch> match(const cv::Mat& img, const cv::Mat& proj_mat, const pfc::matchConfig& cfg);

/**
 * @brief Equivalent to match but divides the pitch search amongst parallel threads
 * @param img the image to match the template to
 * @param proj_mat camera projection matrix for template generation
 * @param max_pitch maximum pitch
 * @param min_pitch minimum pitch
 * @param pitch_inc pitch increment
 * @return vector of template matches run
 */
std::vector<TemplateMatch> matchThreaded(const cv::Mat& img, const cv::Mat& proj_mat,
										 int max_pitch, int min_pitch, int pitch_inc);

/**
 * @brief Run opencv template match
 * @param img img to match template over
 * @param templ template to match
 * @return resulting template match
 */
TemplateMatch getMatch(const cv::Mat &img, const cv::Mat &templ);

/**
 * @brief Generate a template, a binary image, of a semi-circular needle at depth z from the camera,
 * rotated by euler angle rotations r, p, y and projected according to a given projection matrix
 * @param z depth of needle origin from camera
 * @param r roll
 * @param p pitch
 * @param y yaw
 * @param proj_mat projection matrix of simulated camera
 * @return pair[generated template, origin of the needle in the image]
 */
std::pair<cv::Mat, cv::Point2i> generateTemplate(double z, int r, int p, int y, const cv::Mat& proj_mat);

/**
 * @brief
 * @param matches
 * @param num_cand_pts
 * @return
 */
std::vector<TemplateMatch> collect_top_matches(const std::vector<TemplateMatch> &matches, int num_cand_pts);

#endif //PF_INIT_NODE_INCLUDE_PF_INIT_NODE_MATCHING_H_
