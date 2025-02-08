#ifndef ARUCO_HPP
#define ARUCO_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <map>
#include "../utils/utils.hpp"


class Arucos {

    public:
        enum {
            CENTER_TOP_LEFT = 23,
            CENTER_TOP_RIGHT = 22,
            CENTER_BOTTOM_RIGHT = 21,
            CENTER_BOTTOM_LEFT = 20,
            ROBOTS_MIN = 1,
            ROBOTS_MAX = 10
        };

        Arucos(cv::Mat& image);

        cv::Point2f& operator[](int id);
        cv::Point2f& getPosition(int id, bool rawPosition = false);

        void draw(cv::Mat& input);

        void warp(bool updateArucos = true);
        void warp(cv::Mat& output, bool updateArucos = true);

        void getDistortion(int id, cv::Point2f& distortion);

    private:
        static const cv::aruco::Dictionary dictionary;
        static const cv::aruco::DetectorParameters detectorParams;
        static const cv::aruco::ArucoDetector detector;
        static const std::vector<cv::Point2f> dst;

        std::map<int, cv::Point2f> arucos;
        std::map<int, cv::Point2f> realPos;
        cv::Mat& image;
        bool cornersOutdated;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
        cv::Mat transformMatrix;

};

#endif /* ARUCO_ARUCO_HPP */
