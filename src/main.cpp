#include <opencv2/opencv.hpp>
#include <iostream>
#include "aruco/aruco.hpp"
#include "plank/plank.hpp"
#include "utils/utils.hpp"

#define NFRAME -1 /* -1 for all the frames */
#define VIDEO_OFFSET 1 /* non null offset */


int main(int argc, char** argv) {
#ifndef CUDA
    cv::Mat base, frame;
#else
    cv::Mat baseMat, frameMat;
    cv::cuda::GpuMat base, frame;
#endif
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
#ifndef CUDA
    base = cv::imread(argv[1]);
    if (base.empty()) {
        std::cerr << "Could not open or find the base image." << std::endl;
        return 1;
    }
#else
    baseMat = cv::imread(argv[1]);
    if (baseMat.empty()) {
        std::cerr << "Could not open or find the base image." << std::endl;
        return 1;
    }
    base.upload(baseMat);
#endif
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
#ifndef CUDA
        capture >> frame;
#else
        capture >> frameMat;
        frame.upload(frameMat);
#endif
    }

    /* warp the base image */
#ifndef CUDA
    baseArucos.nextFrame(base);
#else
    baseArucos.nextFrame(baseMat);
#endif
    baseArucos.warp(base, base);

    /* warp the first frame to get the tranformation matrix */
    /* By setting up the transformation matrix only once, we assume
    that the camera is fixed. Otherwise, we need to update the matrix
    for each frame which might be computationally expensive. */
#ifndef CUDA
    arucos.nextFrame(frame);
#else
    arucos.nextFrame(frameMat);
#endif
    arucos.warp(frame, frame, false);

    iframe = VIDEO_OFFSET;
    while (NFRAME < 0 || iframe < NFRAME + VIDEO_OFFSET) {

        /* retrieve the frame */
#ifndef CUDA
        capture >> frame;
#else
        capture >> frameMat;
        frame.upload(frameMat);
#endif
        if (frame.empty()) break;

        const auto start = std::chrono::high_resolution_clock::now();

        /* get the arucos position as well as their real position in space
        and warp the frame with the former tranformation matrix */
#ifndef CUDA
        arucos.nextFrame(frame);
#else
        arucos.nextFrame(frameMat);
#endif
        arucos.warp(frame, frame, true);

        /* get the planks */
        planks = Planks::Get(base, frame, arucos);

        const auto end = std::chrono::high_resolution_clock::now();
        elapsed += end - start;

#ifndef CUDA
        arucos.draw(frame);
        Planks::Draw(frame, planks);
        outputVideo.write(frame);
#else
        frame.download(frameMat);
        arucos.draw(frameMat);
        Planks::Draw(frameMat, planks);
        outputVideo.write(frameMat);
#endif
        // arucos.print(std::cout);

        // std::cout << iframe / 60.0f << "s (" << iframe << ")" << std::endl;
        iframe++;
    }

    std::cout << "Avg. processing time " << elapsed.count() / static_cast<double>(iframe - VIDEO_OFFSET) << "s" << std::endl;

    capture.release();
    outputVideo.release();

    return 0;
}
