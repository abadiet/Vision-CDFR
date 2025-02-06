#ifndef ARUCO_ARUCO_HPP
#define ARUCO_ARUCO_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <map>

#define ARUCO_CENTER_TOPLEFT 23
#define ARUCO_CENTER_TOPRIGHT 22
#define ARUCO_CENTER_BOTTOMLEFT 21
#define ARUCO_CENTER_BOTTOMRIGHT 20


class Arucos {

    public:
        Arucos(cv::Mat& image);

        cv::Point2f& operator[](int id);

        void draw(cv::Mat& input);

        void warp(bool updateArucos = true);
        void warp(cv::Mat& output, bool updateArucos = true);

    private:
        static const cv::aruco::Dictionary dictionary;
        static const cv::aruco::DetectorParameters detectorParams;
        static const cv::aruco::ArucoDetector detector;
        static const std::vector<cv::Point2f> dst;

        void detectMarkers();

        std::map<int, cv::Point2f> arucos;
        cv::Mat& image;
        bool cornersOutdated;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
        cv::Mat transformMatrix;

};

#endif /* ARUCO_ARUCO_HPP */
