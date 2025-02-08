#ifndef UTILS_HPP
#define UTILS_HPP

#include <opencv2/opencv.hpp>

#define CAMERA_POS cv::Point2f(1530.0f, 2000.0f) /* TODO to change on a real table */
#define CAMERA_Z 1370.0f
#define ROBOTS_ARUCO_Z 450.0f
#define ROBOTS_RATIO (1 - ROBOTS_ARUCO_Z / CAMERA_Z)

#define UNUSED(x) (void)(x)

void showImage(const std::string& name, cv::Mat& image);

#endif /* UTILS_HPP */
