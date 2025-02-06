#include "aruco.hpp"

void getArucos(cv::Mat& image, arucos_t& arucos, bool draw) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
    unsigned int i;

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detector.detectMarkers(image, corners, ids, rejectedCandidates);

    if (draw) {
        cv::aruco::drawDetectedMarkers(image, corners, ids);
    }

    arucos.clear();
    for (i = 0; i < ids.size(); ++i) {
        cv::Point2f center(0, 0);
        for (cv::Point2f& corner : corners[i]) {
            center += corner;
        }
        center /= 4;

        arucos[ids[i]] = center;
    }
}
