#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "aruco/aruco.hpp"
#include "plank/plank.hpp"
#include "utils/utils.hpp"

#define NFRAME -1
#define VIDEO_OFFSET 1


int main(int argc, char** argv) {
    cv::Mat base, frame;
    std::vector<Planks::plank> planks;
    cv::VideoCapture capture;
    cv::VideoWriter outputVideo;
    Arucos baseArucos, arucos;
    std::chrono::duration<double> elapsed;
    unsigned int iframe;
    // std::vector<std::vector<cv::Point>> contours;

    if (argc != 4) {
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

    /* offset the video */
    for (unsigned int i = 0; i < VIDEO_OFFSET; i++) {
        capture >> frame;
    }

    /* warp the base image */
    baseArucos.nextFrame(base);
    baseArucos.warp(base, base);

    /* warp the first frame to get the tranformation matrix */
    /* By setting up the transformation matrix only once, we assume
    that the camera is fixed. Otherwise, we need to update the matrix
    for each frame which might be computationally expensive. */
    arucos.nextFrame(frame);
    arucos.warp(frame, frame, false);

    iframe = VIDEO_OFFSET;
    while (NFRAME < 0 || iframe < NFRAME + VIDEO_OFFSET) {

        /* retrieve the frame */
        capture >> frame;
        if (frame.empty()) break;

        const auto start = std::chrono::high_resolution_clock::now();

        /* get the arucos position as well as their real position in space
        and warp the frame with the former tranformation matrix */
        arucos.nextFrame(frame);
        arucos.warp(frame, frame, true);

        /* get the planks */
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
