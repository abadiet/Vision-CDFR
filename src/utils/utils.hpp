#ifndef UTILS_HPP
#define UTILS_HPP

#include <opencv2/opencv.hpp>

#define CAMERA_POS cv::Point2f(1470.0f, 0.0f)
#define CAMERA_Z 1370.0f
#define ROBOTS_ARUCO_Z 450.0f

#define PLANK_EPSI 0.95f
#define PLANK_L 400.0f * PLANK_EPSI
#define PLANK_l 100.0f * PLANK_EPSI
#define N_CONTROL_POINTS 18
#define N_CONTROL_POINTS_THRESHOLD 2

#define MIN_DST_ROBOTS_PLANK 200.0f
#define ROBOTS_RADIUS 150
#define ROBOTS_HEIGHT 350.0f

#define UNUSED(x) (void)(x)


void ShowImage(const std::string& name, cv::Mat& image);

#endif /* UTILS_HPP */
