#include "utils.hpp"

void ShowImage(const std::string& name, cv::Mat& image) {
    cv::imshow(name, image);
    cv::waitKey(0);
}
