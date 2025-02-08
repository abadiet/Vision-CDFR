#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "aruco/aruco.hpp"
#include "plank/plank.hpp"
#include "utils/utils.hpp"

#define NTESTS 1


int main(int argc, char** argv) {
    cv::Mat base, image, frame;
    std::vector<Planks::plank> planks;
    // std::vector<std::vector<cv::Point>> contours;

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

    Arucos baseArucos(base);
    baseArucos.warp(base);

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < NTESTS; i++) {
        frame = image.clone();

        Arucos arucos(frame);
        arucos.warp(frame);

        planks = Planks::Get(base, frame, arucos);

        Planks::Draw(frame, planks);
        cv::circle(frame, arucos[6], 5, cv::Scalar(0, 255, 0), -1);
        cv::circle(frame, arucos[6], 150, cv::Scalar(0, 255, 0), 1);
        showImage("Vision", frame);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Execution time: " << elapsed.count() / NTESTS << " seconds" << std::endl;

    return 0;
}
