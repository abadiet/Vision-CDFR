#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include "aruco/aruco.hpp"
#include "plank/plank.hpp"
#include "utils/utils.hpp"


int main(int argc, char** argv) {
    cv::Mat base, image, frame;
    std::vector<Planks::plank> planks;
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

    Arucos baseArucos(base);
    baseArucos.warp(base);

    auto start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < 1; i++) {
        frame = image.clone();

        Arucos arucos(frame);
        arucos.warp(frame);

        planks = Planks::Get(base, frame, arucos, &contours);
        Planks::Draw(frame, planks, &contours);
        cv::imshow("Planks", frame);
        cv::waitKey(0);
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Execution time: " << elapsed.count() / 1.0 << " seconds" << std::endl;

    // printPlanks(image, planks, &contours);
    // cv::imshow("Planks", image);
    // cv::waitKey(0);

    return 0;
}
