#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "aruco/aruco.hpp"
#include "plank/plank.hpp"
#include "utils/utils.hpp"

#define NFRAME 200
#define VIDEO_OFFSET 150


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

    baseArucos.nextFrame(base);
    baseArucos.warp(base, base);

    for (unsigned int i = 0; i < VIDEO_OFFSET; i++) {
        capture >> frame;
    }

    arucos.nextFrame(frame);
    arucos.warp(frame, frame, false);

    iframe = VIDEO_OFFSET;
    while (NFRAME < 0 || iframe < NFRAME + VIDEO_OFFSET) {
        capture >> frame;
        if (frame.empty()) break;

        const auto start = std::chrono::high_resolution_clock::now();

        arucos.nextFrame(frame);
        arucos.warp(frame, frame, true);

        planks = Planks::Get(base, frame, arucos);

        const auto end = std::chrono::high_resolution_clock::now();
        elapsed += end - start;

        arucos.draw(frame);
        Planks::Draw(frame, planks);

        // arucos.print(std::cout);

        outputVideo.write(frame);

        // std::cout << iframe / 60.0f << "s (" << iframe << ")" << std::endl;
        iframe++;
    }

    std::cout << "Avg. processing time " << elapsed.count() / (double) (iframe - VIDEO_OFFSET) << "s" << std::endl;

    capture.release();
    outputVideo.release();

    return 0;
}
