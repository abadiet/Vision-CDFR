#include "aruco.hpp"


const cv::aruco::Dictionary Arucos::dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
const cv::aruco::DetectorParameters Arucos::detectorParams = cv::aruco::DetectorParameters();
const cv::aruco::ArucoDetector Arucos::detector = cv::aruco::ArucoDetector(dictionary, detectorParams);
const std::vector<cv::Point2f> Arucos::dst = {
    cv::Point2f(2400, 1400),
    cv::Point2f(600, 1400),
    cv::Point2f(600, 600),
    cv::Point2f(2400, 600)
};/* TODO to change on a real table
    cv::Point2f(2400, 600),
    cv::Point2f(600, 600),
    cv::Point2f(2400, 1400),
    cv::Point2f(600, 1400)
}*/

Arucos::Arucos(cv::Mat& image) :
    image(image)
{
    unsigned int i;

    detector.detectMarkers(image, corners, ids, rejectedCandidates);

    for (i = 0; i < ids.size(); ++i) {
        cv::Point2f center(0, 0);
        for (cv::Point2f& corner : corners[i]) {
            center += corner;
        }
        center /= 4;

        arucos[ids[i]] = center;
    }
}

cv::Point2f& Arucos::operator[](int id) {
    if (arucos.find(id) == arucos.end()) {
        throw std::out_of_range("Arucos::operator[] for id " + std::to_string(id));
    }
    return arucos[id];
}

void Arucos::draw(cv::Mat& input) {
    if (cornersOutdated) {
        for (const std::vector<cv::Point2f>& c : corners) {
            cv::perspectiveTransform(c, c, transformMatrix);
        }
        cornersOutdated = false;
    }
    cv::aruco::drawDetectedMarkers(input, corners, ids);
}

void Arucos::warp(bool updateArucos) {
    warp(image, updateArucos);
}

void Arucos::warp(cv::Mat& output, bool updateArucos) {
    std::vector<cv::Point2f> src(4), centers;
    unsigned int i;

    try {
        src[0] = (*this)[Arucos::CENTER_TOP_LEFT];
        src[1] = (*this)[Arucos::CENTER_TOP_RIGHT];
        src[2] = (*this)[Arucos::CENTER_BOTTOM_LEFT];
        src[3] = (*this)[Arucos::CENTER_BOTTOM_RIGHT];
    } catch (const std::out_of_range& e) {
        throw std::runtime_error("Not all center markers found!");
    }

    transformMatrix = cv::getPerspectiveTransform(src, dst);
    cv::warpPerspective(image, output, transformMatrix, cv::Size(3000, 2000));

    if (updateArucos) {
        for (const std::pair<const int, cv::Point2f>& aruco : arucos) {
            centers.push_back(aruco.second);
        }
        cv::perspectiveTransform(centers, centers, transformMatrix);
        i = 0;
        for (std::pair<const int, cv::Point2f>& aruco : arucos) {
            if (aruco.first < Arucos::ROBOTS_MIN || aruco.first > Arucos::ROBOTS_MAX) {
                aruco.second = centers[i++];
            } else {
                aruco.second = CAMERA_POS + (centers[i] - CAMERA_POS) * ROBOTS_RATIO;
                i++;
            }
        }

        cornersOutdated = true;
    }
}
