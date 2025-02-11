#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "aruco/aruco.hpp"
#include "plank/plank.hpp"
#include "utils/utils.hpp"

#define NFRAME 1000


int main(int argc, char** argv) {
    cv::Mat base, frame;
    std::vector<Planks::plank> planks;
    cv::VideoCapture capture;
    cv::VideoWriter outputVideo;
    Arucos baseArucos, arucos;
    std::chrono::duration<double> elapsed;
    unsigned int iframe;
    // std::vector<std::vector<cv::Point>> contours;

    if ( argc != 4 ) {
        std::cout << "usage: Vision <base_image> <input_video> <output_video>" << std::endl;
        return 1;
    }
    base = cv::imread(argv[1]);
    if (base.empty()) {
        std::cerr << "Could not open or find the base image." << std::endl;
        return 1;
    }
    capture = cv::VideoCapture(argv[2]);
    if (!capture.isOpened()) {
        std::cerr << "Could not open or find the video." << std::endl;
        return 1;
    }
    outputVideo.open(
        argv[3],
        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
        60,
        cv::Size(3000, 2000),
        true
    );
    if (!outputVideo.isOpened()) {
        std::cerr << "Could not open the output video for write." << std::endl;
        return 1;
    }

    baseArucos.get(base);
    baseArucos.warp(base, base);

    capture >> frame;
    arucos.get(frame);
    arucos.warp(frame, frame, true, false);

    iframe = 0;
    while (NFRAME < 0 || iframe < NFRAME) {
        capture >> frame;
        if (frame.empty()) break;

        const auto start = std::chrono::high_resolution_clock::now();

        arucos.get(frame);
        arucos.warp(frame, frame, true, true);

        planks = Planks::Get(base, frame, arucos);

        const auto end = std::chrono::high_resolution_clock::now();
        elapsed += end - start;

        arucos.draw(frame);
        Planks::Draw(frame, planks);

        outputVideo.write(frame);

        iframe++;
    }

    std::cout << "Avg. processing time " << elapsed.count() / (double) iframe << "s" << std::endl;

    capture.release();
    outputVideo.release();

    return 0;
}
