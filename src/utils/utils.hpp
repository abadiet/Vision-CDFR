#ifndef UTILS_HPP
#define UTILS_HPP

#include <opencv2/opencv.hpp>

#define CAMERA_POS cv::Point2f(1550.0f, -50.0f)
#define CAMERA_Z 1370.0f
#define ROBOTS_ARUCO_Z 450.0f

#define ARUCO_NOTFOUND_THRESHOLD 30

#define PLANK_EPSI 0.95f
#define PLANK_L 400.0f * PLANK_EPSI
#define PLANK_l 100.0f * PLANK_EPSI
#define N_CONTROL_POINTS 18
#define N_CONTROL_POINTS_THRESHOLD 2

#define MIN_DST_ROBOTS_PLANK 200.0f
#define ROBOTS_DIAMETER 300
#define ROBOTS_HEIGHT 350.0f

#define UNUSED(x) (void)(x)

#ifndef CUDA
typedef cv::Mat Mat;
#else
typedef cv::gpu::GpuMat Mat;
#endif

/**
 * @brief Display an image in a window
 * @param name: the name of the window
 * @param image: the image to display
 */
void ShowImage(const std::string& name, cv::Mat& image);

#endif /* UTILS_HPP */
