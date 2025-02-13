#ifndef PLANK_HPP
#define PLANK_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "../aruco/aruco.hpp"
#include "../utils/utils.hpp"
#ifdef CUDA
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#endif


/**
 * @brief Static class for planks detection
 */
class Planks {

    public:
        /**
         * @brief A plank structure
         */
        struct plank {
            cv::Point2f center;
            cv::Point2f direction;
        };

        // static void RemoveInitPlanks(cv::Mat& image);

        /**
         * @brief Get the planks from the image
         * @param base the base image
         * @param image the image to process
         * @param arucos the arucos on the image
         * @param contours retrieve the contours detected on the image
         * @return the planks detected
         */
        static std::vector<plank> Get(Mat& base, Mat& image, Arucos& arucos, std::vector<std::vector<cv::Point>>* contours = nullptr);

        /**
         * @brief Draw the planks on the image
         * @param image the image to draw on
         * @param planks the planks to draw
         * @param contours the contours to draw
         */
        static void Draw(cv::Mat& image, std::vector<plank>& planks, std::vector<std::vector<cv::Point>>* contours = nullptr);

        /**
         * @brief Print the planks
         * @param os the output stream
         * @param planks the planks to print
         */
        static void Print(std::ostream& os, std::vector<plank>& planks);

        /**
         * @brief Get the filtered image
         * @param base the base image
         * @param image the image to process
         * @param arucos the arucos on the image
         * @param filtered the resulting filtered image
         */
        static void GetFilteredImage(Mat& base, Mat& image, Arucos& arucos, Mat& filtered);

    private:

        /* buffer */
        static Mat filtered;
#ifdef CUDA
        static cv::Mat filteredMat;
#endif

        /* filters' kernels */
        static const cv::Mat openKernel, closeKernel;
#ifdef CUDA
        static const cv::Ptr<cv::cuda::Filter> morphOpen, morphClose;
#endif

};

#endif /* PLANK_HPP */
