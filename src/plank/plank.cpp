#include "plank.hpp"

#define PLANK_WIDTH 100
#define PLANK_HEIGHT 400
#define MIN_PLANK_AREA PLANK_WIDTH * PLANK_HEIGHT - 10
#define MAX_PLANK_AREA PLANK_WIDTH * PLANK_HEIGHT * 1.5 /* TODO */


void getFilteredImage(cv::Mat& base, cv::Mat& image, cv::Mat& filtered) {
    cv::Mat mask;

    cv::absdiff(base, image, filtered);
    cv::cvtColor(filtered, filtered, cv::COLOR_BGR2GRAY);
    cv::morphologyEx(filtered, filtered, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    cv::threshold(filtered, filtered, 50, 255, cv::THRESH_BINARY);
    cv::morphologyEx(filtered, filtered, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(61, 61)));
    cv::threshold(filtered, filtered, 25, 255, cv::THRESH_BINARY);
}

std::vector<Planks::plank> Planks::Get(cv::Mat& base, cv::Mat& image, std::vector<std::vector<cv::Point>>* contours) {
    cv::Mat filtered, canny_output;
    std::vector<Planks::plank> planks;
    Planks::plank plank;
    std::vector<std::vector<cv::Point>> conts;

    getFilteredImage(base, image, filtered);

    cv::Canny(filtered, canny_output, 250, 250);
    cv::findContours(canny_output, conts, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for (std::vector<cv::Point>& contour : conts) {
        cv::approxPolyDP(contour, contour, 20, true);
        if (contour.size() == 4 && cv::isContourConvex(contour)) {
            const double area = cv::contourArea(contour);
            if (area >= MIN_PLANK_AREA && area <= MAX_PLANK_AREA) {
                plank.center = (contour[0] + contour[1] + contour[2] + contour[3]) / 4.0f;
                const cv::Point2f direction1 = (contour[1] - contour[0]);
                const cv::Point2f direction2 = (contour[2] - contour[1]);
                const double norm1 = cv::norm(direction1);
                const double norm2 = cv::norm(direction2);
                if (norm1 > norm2) {
                    plank.direction = direction1 / norm1;
                } else {
                    plank.direction = direction2 / norm2;
                }
                planks.push_back(plank);

                if (contours != nullptr) {
                    contours->push_back(contour);
                }
            }
        }
    }

    return planks;
}

void Planks::Draw(cv::Mat& image, std::vector<Planks::plank>& planks, std::vector<std::vector<cv::Point>>* contours) {
    for (Planks::plank& plank : planks) {
        cv::line(image, plank.center, plank.center + plank.direction * 100, cv::Scalar(0, 0, 255), 2);
        std::cout << "Plank at " << plank.center << " with direction " << plank.direction << std::endl;
    }
    if (contours != nullptr) {  
        cv::drawContours(image, *contours, -1, cv::Scalar(0, 255, 0), 2);
    }
}
