#ifndef PLANK_HPP
#define PLANK_HPP

#include <opencv2/opencv.hpp>
#include <vector>


typedef struct {
    cv::Point2f center;
    cv::Point2f direction;
} plank_t;

void getPlanks(cv::Mat& base, cv::Mat& image, std::vector<plank_t>& planks, std::vector<std::vector<cv::Point>>* contours = nullptr);
void printPlanks(cv::Mat& image, std::vector<plank_t>& planks, std::vector<std::vector<cv::Point>>* contours = nullptr);

#endif /* PLANK_HPP */
