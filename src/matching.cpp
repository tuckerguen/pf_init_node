#include "util.h"
#include <future>
#include "matching.h"
#include <sys/sysinfo.h>

std::vector<TemplateMatch> match(const cv::Mat& img, const cv::Mat& proj_mat, const pfc::matchConfig& cfg) {
	// Initialize min heap priority queue
	std::priority_queue<TemplateMatch, std::vector<TemplateMatch>, TemplateMatchComparator> best_matches;

	// Convert z to an integer based loop
	int n_iter = (int)ceil((cfg.max_z - cfg.min_z) / cfg.z_inc);
	for (int i = 0; i < n_iter; i++) {
		// Enumerate rotation options
		for (int y = cfg.min_yaw; y <= cfg.max_yaw; y += cfg.yaw_inc) {
			for (int p = cfg.min_pitch; p <= cfg.max_pitch; p += cfg.pitch_inc) {
				for (int r = cfg.min_roll; r <= cfg.max_roll; r += cfg.roll_inc) {
					// Convert z iterator back to double
					double z = cfg.min_z + i*cfg.z_inc;

					// Generate Template
					std::pair<cv::Mat, cv::Point2i> templ_origin_pair = generateTemplate(z, r, p, y, proj_mat);
					//Match rotated template to image
					TemplateMatch new_match = getMatch(img, templ_origin_pair.first);

					// Store other match details
					new_match.z = z;
					new_match.yaw = constrainAngle(y, false);
					new_match.pitch = constrainAngle(p, false);
					new_match.roll = constrainAngle(r, false);

					// Offset the origin to sit in the correct location of the final image
					new_match.origin = templ_origin_pair.second + cv::Point2i(new_match.rect.x, new_match.rect.y);

					// If queue not full
					if (best_matches.size() < cfg.num_cand_pts) {
						// Insert match
						best_matches.push(new_match);
					}
					else if (new_match.score > best_matches.top().score) {
						// If queue full and score is better than min score
						// Delete the smallest score match
						best_matches.pop();
						// Add the new better match
						best_matches.push(new_match);
					}
				}
			}
		}
	}
	return pq_to_vector(best_matches);
}

std::vector<TemplateMatch> matchThreaded(const cv::Mat& img, const cv::Mat& proj_mat,
										 int max_pitch, int min_pitch, int pitch_inc) {
	// Ensure the range the threads will divide over divides evenly
	assert(((max_pitch - min_pitch) / pitch_inc) == 0);

	// Init thread matches and thread return values (futures)
	std::vector<std::future<std::vector<TemplateMatch>>> futures;

	// Calc num threads and scale increment between sequential threads
	int num_threads = get_nprocs();
	double thread_inc = ((max_pitch - min_pitch) / (double)num_threads);

	// Copy the default parameters
	pfc::matchConfig thread_cfg = pfc::cfg;
	// Launch threads
	for (int tid = 0; tid < num_threads; tid++) {
		// Compute thread range
		int minp = (int)std::floor(min_pitch + (tid * thread_inc));
		int maxp = (int)std::ceil(min_pitch + ((tid + 1) * thread_inc));
		minp -= minp % pitch_inc;
		maxp -= maxp % pitch_inc;

		// Except first, shift range to avoid overlap
		if (tid!=0) {
			thread_cfg.min_pitch += pitch_inc;
		}
		thread_cfg.min_pitch = minp;
		thread_cfg.max_pitch = maxp;

		// Launch thread and collect future
		futures.push_back(std::async(std::launch::async, match, img, proj_mat, thread_cfg));
	}

	// Extract matches from futures
	std::vector<TemplateMatch> all_matches;
	for (auto &fut : futures) {
		std::vector<TemplateMatch> thread_matches = fut.get();
		all_matches.insert(all_matches.end(), thread_matches.begin(), thread_matches.end());
	}

	return collect_top_matches(all_matches, pfc::cfg.num_cand_pts);
}

TemplateMatch getMatch(const cv::Mat &img, const cv::Mat &templ) {
	// Match using TM_CCOEFF_NORMED
	cv::Mat result;
	cv::matchTemplate(img, templ, result, cv::TM_CCOEFF_NORMED);

	/// Localizing the best match with minMaxLoc
	double minVal;
	double maxVal;
	cv::Point minLoc;
	cv::Point maxLoc;
	cv::Point matchLoc;
	cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

	// Create template match
	TemplateMatch match(maxVal, cv::Rect2i(maxLoc.x, maxLoc.y, templ.cols, templ.rows));

	return match;
}


std::pair<cv::Mat, cv::Point2i> generateTemplate(double z, int r, int p, int y, const cv::Mat &proj_mat) {
	// Array of needle points
	std::vector<cv::Point2i> needle_arc;

	// Transformation Matrix
	// Formula for general rotation matrix from: https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
	cv::Mat transform = (cv::Mat_<double>(4, 4) <<
												cos(y)*cos(p), cos(y)*sin(p)*sin(r) - sin(y)*cos(r),
		cos(y)*sin(p)*cos(r) + sin(y)*sin(r), 0.0,
		sin(y)*cos(p), sin(y)*sin(p)*sin(r) + cos(y)*cos(r), sin(y)*sin(p)*cos(r) - cos(y)*sin(r), 0.0,
		-sin(p), cos(p)*sin(r), cos(p)*cos(r), z,
		0.0, 0.0, 0.0, 1.0
	);


	// To determine bounding box of needle for cropping
	// Defaults are max resolution of DaVinci endoscopes (640x480)
	int leftmost = pfc::endo_x_res;
	int rightmost = 0;
	int upmost = pfc::endo_y_res;
	int downmost = 0;
	int buffer = 5;

	// Calculate the transformed origin
	cv::Mat needle_arc_origin = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
	cv::Point2d origin = calcUVPoint(transform * needle_arc_origin, proj_mat);

	// Calculate transformed arc points
	for (int i = 0; i <= pfc::cfg.resolution; i++) {
		double turn_angle = i*(M_PI/pfc::cfg.resolution);

		cv::Mat needle_arc_3d_pt = (cv::Mat_<double>(4, 1) <<
														   pfc::radius*cos(turn_angle),
			pfc::radius*sin(turn_angle),
			0.0,
			1.0
		);

		cv::Point2i uv_pt = calcUVPoint(transform * needle_arc_3d_pt, proj_mat);
		needle_arc.push_back(uv_pt);

		// Track the needle boundaries
		leftmost = std::min(uv_pt.x, leftmost);
		rightmost = std::max(uv_pt.x, rightmost);
		upmost = std::min(uv_pt.y, upmost);
		downmost = std::max(uv_pt.y, downmost);
	}

	// Draw into template
	cv::Mat templ(cv::Size(pfc::endo_x_res, pfc::endo_y_res), CV_8U, cv::Scalar(0));
	for (int i = 1; i <= pfc::cfg.resolution; i++) {
		cv::Point2d p1 = needle_arc.at(i - 1);
		cv::Point2d p2 = needle_arc.at(i);

		cv::line(templ, p1, p2, cv::Scalar(255), 3, 8, 0);
	}

	// Crop image to only include the needle
	upmost = (int)std::floor(upmost);
	leftmost = (int)std::floor(leftmost);
	downmost = (int)std::ceil(downmost);
	rightmost = (int)std::ceil(rightmost);
	cv::Range rows(upmost - buffer, downmost + buffer);
	cv::Range cols(leftmost - buffer, rightmost + buffer);

	// Crop the template around the needle
	templ = templ(rows, cols);
	//Adjust origin to be relative to new size
	origin.x -= leftmost;
	origin.y -= upmost;

	std::pair<cv::Mat, cv::Point2i> templ_origin_pair(templ, origin);
	return templ_origin_pair;
}

std::vector<TemplateMatch> collect_top_matches(const std::vector<TemplateMatch> &matches, int num_cand_pts) {
	std::priority_queue<TemplateMatch, std::vector<TemplateMatch>, TemplateMatchComparator> best_matches;

	for (const auto &m : matches) {
		// If queue not full
		if (best_matches.size() < num_cand_pts) {
			// Insert match
			best_matches.push(m);
		}
		// If queue full and score is better than min score
		else if (m.score > best_matches.top().score) {
			// Delete the smallest score match
			best_matches.pop();
			// Add the new better match
			best_matches.push(m);
		}
		// Otherwise reject the match since it isn't in the top n scores
	}
	return pq_to_vector(best_matches);
}