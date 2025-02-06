#ifndef ARUCO_ARUCO_HPP
#define ARUCO_ARUCO_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <map>

#define ARUCO_CENTER_TOPLEFT 23
#define ARUCO_CENTER_TOPRIGHT 22
#define ARUCO_CENTER_BOTTOMLEFT 21
#define ARUCO_CENTER_BOTTOMRIGHT 20

typedef std::map<int, cv::Point2f> arucos_t;

void getArucos(cv::Mat& image, arucos_t& arucos, bool draw = false);

#endif /* ARUCO_ARUCO_HPP */
