#ifndef ARUCO_HPP
#define ARUCO_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <map>
#include "../utils/utils.hpp"
#ifdef CUDA
#include <opencv2/cudawarping.hpp>
#endif

#define ARUCO_POS_MEMORY 5  /* to adjust relatively to the speed of the moving objects */


/**
 * @brief Class for arucos detection
 */
class Arucos {

    public:
        /**
         * @brief Aruco ids
         */
        enum {
            CENTER_TOP_LEFT = 23,
            CENTER_TOP_RIGHT = 22,
            CENTER_BOTTOM_RIGHT = 21,
            CENTER_BOTTOM_LEFT = 20,
            ROBOTS_MIN = 1,
            ROBOTS_MAX = 10
        };

        /**
         * @brief Element structure
         */
        struct element {
            cv::Point2f raw; /* position on raw image */
            cv::Point2f aruco; /* position on wrapped image */
            cv::Point2f real[ARUCO_POS_MEMORY]; /* latest real positions in space */
            unsigned int notFound; /* number of consecutives times the aruco has not been find */
        };

        /**
         * @brief Constructor
         */
        Arucos();

        /**
         * @brief Process the next frame to update the arucos positions
         * @param image the image to process
         */
        void nextFrame(cv::Mat& image);

        /**
         * @brief Get the real position of an element
         * @param id the id of the aruco
         * @return the position of the element
         */
        cv::Point2f& operator[](int id);

        /**
         * @brief Get the position of an element
         * @param id the id of the aruco
         * @param projected if true, return the position on the wrapped image, otherwise return the position the raw position
         * @param aruco if true, return the position of the aruco, otherwise return the real position of the element
         * @return reference to the position of the element
         */
        cv::Point2f& getPosition(int id, bool projected = true, bool aruco = false);

        /**
         * @brief Warp the input image to the output image
         * @param input the input image
         * @param output the output image
         * @param usePreviousMatrix if true, use the previous transformation matrix
         * @param forceUpdateMatrix if true, force the update of the transformation matrix
         */
        void warp(Mat& input, Mat& output, bool usePreviousMatrix = false, bool forceUpdateMatrix = false);

        /**
         * @brief Get the distortion of the element id
         * @param id the id of the aruco
         * @param distortion the distortion of the element
         */
        void getDistortion(int id, cv::Point2f& distortion);

        /**
         * @brief Draw the arucos as well as the robots on the input image
         * @param input the input image
         */
        void draw(cv::Mat& input);

        /**
         * @brief Print the arucos
         * @param os the output stream
         */
        void print(std::ostream& os);

    private:
        static const cv::aruco::Dictionary dictionary;
        static const cv::aruco::DetectorParameters detectorParams;
        static const cv::aruco::ArucoDetector detector;
        static const std::vector<cv::Point2f> dst;

        std::map<int, struct element> elements;
        bool cornersOutdated;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejectedCandidates;
        cv::Mat transformMatrix;

};

#endif /* ARUCO_ARUCO_HPP */
