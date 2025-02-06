#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "aruco/aruco.hpp"
#include "plank/plank.hpp"


void warpImage(cv::Mat& image, arucos_t& arucos, cv::Mat& warpedImage);


int main(int argc, char** argv) {
    cv::Mat base, image;
    arucos_t arucos;
    std::vector<plank_t> planks;
    std::vector<std::vector<cv::Point>> contours;

    if ( argc != 3 ) {
        std::cout << "usage: Vision <base_image> <image>" << std::endl;
        return 1;
    }
    base = cv::imread(argv[1]);
    if (base.empty()) {
        std::cerr << "Could not open or find the base image." << std::endl;
        return 1;
    }
    image = cv::imread(argv[2]);
    if (image.empty()) {
        std::cerr << "Could not open or find the image." << std::endl;
        return 1;
    }

    getArucos(image, arucos);
    warpImage(image, arucos, image);

    getArucos(base, arucos);
    warpImage(base, arucos, base);

    getPlanks(base, image, planks, &contours);

    printPlanks(image, planks, &contours);
    cv::imshow("Planks", image);
    cv::waitKey(0);

    return 0;
}


void warpImage(cv::Mat& image, arucos_t& arucos, cv::Mat& warpedImage) {
    std::vector<cv::Point2f> src(4), dst(4);
    cv::Mat transformMatrix;

    if (
        arucos.find(ARUCO_CENTER_TOPLEFT) == arucos.end() ||
        arucos.find(ARUCO_CENTER_TOPRIGHT) == arucos.end() ||
        arucos.find(ARUCO_CENTER_BOTTOMLEFT) == arucos.end() ||
        arucos.find(ARUCO_CENTER_BOTTOMRIGHT) == arucos.end()
    ) {
        throw std::runtime_error("Not all center markers found!");
    }

    src[0] = arucos[ARUCO_CENTER_TOPLEFT];
    src[1] = arucos[ARUCO_CENTER_TOPRIGHT];
    src[2] = arucos[ARUCO_CENTER_BOTTOMLEFT];
    src[3] = arucos[ARUCO_CENTER_BOTTOMRIGHT];

    dst[0] = cv::Point2f(2400, 600);
    dst[1] = cv::Point2f(600, 600);
    dst[2] = cv::Point2f(2400, 1400);
    dst[3] = cv::Point2f(600, 1400);

    transformMatrix = cv::getPerspectiveTransform(src, dst);
    cv::warpPerspective(image, warpedImage, transformMatrix, cv::Size(3000, 2000));
}
