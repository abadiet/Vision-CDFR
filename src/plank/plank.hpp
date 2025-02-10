#ifndef PLANK_HPP
#define PLANK_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "../aruco/aruco.hpp"
#include "../utils/utils.hpp"


class Planks {

    public:
        struct plank {
            cv::Point2f center;
            cv::Point2f direction;
        };

        // static void RemoveInitPlanks(cv::Mat& image);
        static std::vector<plank> Get(cv::Mat& base, cv::Mat& image, Arucos& arucos, std::vector<std::vector<cv::Point>>* contours = nullptr);
        static void Draw(cv::Mat& image, std::vector<plank>& planks, std::vector<std::vector<cv::Point>>* contours = nullptr);

    private:
        /* buffers */
        static cv::Mat filtered, canny_output;
};

#endif /* PLANK_HPP */
