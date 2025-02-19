#include "plank.hpp"

#define PLANK_DIAGONAL static_cast<float> (sqrt(PLANK_L * PLANK_L + PLANK_l * PLANK_l))
#define PLANK_DIAGONAL_ANGLE static_cast<float> (atan(PLANK_l / PLANK_L))
#define PLANK_DIAGONAL_ANGLE_COS static_cast<float> (cos(PLANK_DIAGONAL_ANGLE))
#define PLANK_DIAGONAL_ANGLE_SIN static_cast<float> (sin(PLANK_DIAGONAL_ANGLE))
#define MIN_PLANK_AREA static_cast<double> (PLANK_L * PLANK_l * 0.8f)
#define PLANK_SAME_MAX_DST static_cast<double> (PLANK_l)
#define PLANK_SAME_MAX_TAN_ANG static_cast<double> (tan(0.6)) /* 35deg */
#define ROBOTS_RATIO ROBOTS_HEIGHT / ROBOTS_ARUCO_Z
// #define INIT_PLANKS_RATIO 1.2f

// const std::vector initPlanksPos({
//     std::vector({cv::Point2f(2925.0f, 400.0f), cv::Point2f(0.0f, 1.0f)}),
//     std::vector({cv::Point2f(2925.0f, 1320.0f), cv::Point2f(0.0f, 1.0f)}),
//     std::vector({cv::Point2f(2225.0f, 250.0f), cv::Point2f(1.0f, 0.0f)}),
//     std::vector({cv::Point2f(2175.0f, 1725.0f), cv::Point2f(1.0f, 0.0f)}),
//     std::vector({cv::Point2f(1900.0f, 950.0f), cv::Point2f(1.0f, 0.0f)}),
//     std::vector({cv::Point2f(75.0f, 400.0f), cv::Point2f(0.0f, 1.0f)}),
//     std::vector({cv::Point2f(75.0f, 1320.0f), cv::Point2f(0.0f, 1.0f)}),
//     std::vector({cv::Point2f(775.0f, 250.0f), cv::Point2f(1.0f, 0.0f)}),
//     std::vector({cv::Point2f(825.0f, 1725.0f), cv::Point2f(1.0f, 0.0f)}),
//     std::vector({cv::Point2f(1100.0f, 950.0f), cv::Point2f(1.0f, 0.0f)})
// });


/* buffer */
cv::Mat Planks::filtered;
#ifdef CUDA
cv::cuda::GpuMat Planks::filteredGpu;
#endif

/* filters */
const cv::Mat Planks::openKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
const cv::Mat Planks::closeKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(41, 41));


/**
 * @brief Check if p1 can be the corner of a plank with p1p2 as its direction
 * @param plank the resulting plank
 * @param p1 the first point
 * @param p2 the second point
 * @param contour the contour to check
 * @param Nchecks the number of checks per diagonal
 * @param threshold the threshold of missed checks
 */
bool FindPossiblePlank(Planks::plank& plank, const cv::Point2f& p1, const cv::Point2f& p2, const std::vector<cv::Point>& contour, unsigned int Nchecks, unsigned int threshold);


void Planks::GetFilteredImage(Mat& base, Mat& image, Arucos& arucos, cv::Mat& filtered) {
    cv::Point2f distortion;
    unsigned int i;

    /* difference between the images */
#ifndef CUDA
    cv::absdiff(base, image, filtered);
    cv::cvtColor(filtered, filtered, cv::COLOR_BGR2GRAY);
    cv::threshold(filtered, filtered, 50, 255, cv::THRESH_BINARY);
#else
    cv::cuda::absdiff(base, image, filteredGpu);
    cv::cuda::cvtColor(filteredGpu, filteredGpu, cv::COLOR_BGR2GRAY);
    cv::cuda::threshold(filteredGpu, filteredGpu, 50, 255, cv::THRESH_BINARY);
    filteredGpu.download(filtered);
#endif

    /* hide the robots */
    for (i = Arucos::ROBOTS_MIN; i <= Arucos::ROBOTS_MAX; i++) {
        try {
            arucos.getDistortion(static_cast<int> (i), distortion);
            const cv::Point2f& pos = arucos[static_cast<int> (i)];
            cv::line(filtered, pos, pos + distortion * ROBOTS_RATIO, cv::Scalar(0), ROBOTS_DIAMETER);
        } catch (const std::exception& e) {
            /* robot not found */
            UNUSED(e);
        }
    }

    /* remove the alone pixels and fill the surfaces */
    cv::morphologyEx(filtered, filtered, cv::MORPH_OPEN, Planks::openKernel);
    cv::morphologyEx(filtered, filtered, cv::MORPH_CLOSE, Planks::closeKernel);
}

bool FindPossiblePlank(Planks::plank& plank, const cv::Point2f& p1, const cv::Point2f& p2, const std::vector<cv::Point>& contour, unsigned int Nchecks, unsigned int threshold) {
    unsigned int missed, i;
    short int rectSens, diagSens;
    const float step = PLANK_DIAGONAL / static_cast<float> (Nchecks);
    const cv::Point2f p12 = p1 - p2;

    /* test both sides of the line p1-p2 */
    for (rectSens = -1; rectSens <= 1; rectSens += 2) {
        missed = 0;

        /* test both diagonals of a side */
        for (diagSens = -1; diagSens <= 1; diagSens += 2) {
            const cv::Point2f line = diagSens * p12;
            const cv::Point2f lineNorm = line / cv::norm(line);
            const cv::Point2f start = (diagSens < 0) ? p1 : p1 - lineNorm * PLANK_L;
            const cv::Point2f direction = cv::Point2f(
                lineNorm.x * PLANK_DIAGONAL_ANGLE_COS - lineNorm.y * diagSens * rectSens * PLANK_DIAGONAL_ANGLE_SIN,
                lineNorm.x * diagSens * rectSens * PLANK_DIAGONAL_ANGLE_SIN + lineNorm.y * PLANK_DIAGONAL_ANGLE_COS
            );

            i = 0;
            /* for each test points */
            while (i < Nchecks && missed < threshold) {
                const cv::Point2f p = start + direction * static_cast<float> (i) * step;
                if (cv::pointPolygonTest(contour, p, false) < 0) {
                    missed++;
                    if (missed >= threshold) {
                        /* too many test points are out, this is not a
                        plank */
                        break;
                    }
                }
                i++;
            }

            if (missed >= threshold) {
                /* too many test points are out, this is not a plank */
                break;
            }
        }

        if (missed < threshold) {
            /* a plank is found */
            const cv::Point2f line = p12 / cv::norm(p12);
            plank.center = p1 - line * PLANK_L / 2.0f + cv::Point2f(-1.0f * rectSens * line.y, rectSens * line.x) * PLANK_l / 2.0f;
            /* normalization of the direction */
            if (line.x < 0.0f || (line.x == 0.0f && line.y < 0.0f)) {
                plank.direction = -line;
            } else {
                plank.direction = line;
            }
            return true;
        }
    }
    return false;
}

// void Planks::RemoveInitPlanks(cv::Mat& image) {
//     for (const std::vector<cv::Point2f>& plank : initPlanksPos) {
//         const cv::Point2f& center = plank[0];
//         const cv::Point2f& halfPlank = plank[1] / cv::norm(plank[1]) * PLANK_L * INIT_PLANKS_RATIO / 2.0f;
//         cv::line(image, center, center + halfPlank, cv::Scalar(0), static_cast<int> ((PLANK_l * INIT_PLANKS_RATIO)));
//         cv::line(image, center, center - halfPlank, cv::Scalar(0), static_cast<int> ((PLANK_l * INIT_PLANKS_RATIO)));
//     }
// }

std::vector<Planks::plank> Planks::Get(Mat& base, Mat& image, Arucos& arucos, std::vector<std::vector<cv::Point>>* contours) {
    std::vector<Planks::plank> planks;
    Planks::plank plk;
    std::vector<std::vector<cv::Point>> conts;
    std::vector<cv::Point2f> robotsPos;
    unsigned int i, j;

    /* get the filtered image */
    GetFilteredImage(base, image, arucos, filtered);

    /* get the contours */
    cv::findContours(filtered, conts, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    /* for each contour */
    for (std::vector<cv::Point>& contour : conts) {

        /* approximate the contour to a polynom */
        cv::approxPolyDP(contour, contour, 30, true);

        /* if the area is big enough */
        if (cv::contourArea(contour) >= MIN_PLANK_AREA) {

            /* for each segment of the contour */
            for (i = 0; i < contour.size(); i++) {
                const cv::Point2f& p1 = contour[i];
                const cv::Point2f& p2 = contour[(i + 1) % contour.size()];

                /* check if p1 can be the corner of a plank with p1p2 as its direction */
                if (FindPossiblePlank(plk, p1, p2, contour, N_CONTROL_POINTS, N_CONTROL_POINTS_THRESHOLD)) {
                    /* found one */
                    planks.push_back(plk);

                    if (contours != nullptr) {
                        contours->push_back(contour);
                    }
                }
                /* check if p2 can be the corner of a plank with p2p1 as its direction */
                if (FindPossiblePlank(plk, p2, p1, contour, N_CONTROL_POINTS, N_CONTROL_POINTS_THRESHOLD)) {
                    /* found one */
                    planks.push_back(plk);

                    if (contours != nullptr) {
                        contours->push_back(contour);
                    }
                }
            }
        }
    }

    /* get the robots positions */
    for (i = Arucos::ROBOTS_MIN; i <= Arucos::ROBOTS_MAX; i++) {
        try {
            const cv::Point2f& arucoCenter = arucos[static_cast<int> (i)];
            robotsPos.push_back(arucoCenter);
        } catch (const std::exception& e) {
            /* robot not found */
            UNUSED(e);
        }
    }

    /* remove the robots detected as planks and average the close planks */
    for (i = 0; i < planks.size(); i++) {
        Planks::plank& plank = planks[i];

        /* check if a plank is close to a robot */
        j = 0;
        while (j < robotsPos.size() && cv::norm(robotsPos[j] - plank.center) >= MIN_DST_ROBOTS_PLANK) { /* TODO test irl */
            j++;
        }
        if (j < robotsPos.size()) {
            /* if so, remove it */
            planks.erase(planks.begin() + i);
            i--;
        } else {

            /* average the close planks */
            for (j = i + 1; j < planks.size(); j++) {
                const Planks::plank& p = planks[j];

                /* if p is close to planks[j] */
                if (cv::norm(p.center - plank.center) < PLANK_SAME_MAX_DST) {
                    if (cv::norm(p.direction - plank.direction) < PLANK_SAME_MAX_TAN_ANG) {
                        /* planks[j] is now the average of both and
                        remove p */
                        plank.center = (plank.center + p.center) / 2;
                        plank.direction = (plank.direction + p.direction) / 2;
                        planks.erase(planks.begin() + j);
                        j--;
                    } else {
                        if (cv::norm(p.direction + plank.direction) < 0.1) {
                            /* planks[j] is now the average of both and
                            remove p */
                            plank.center = (plank.center + p.center) / 2;
                            plank.direction = (plank.direction - p.direction) / 2;
                            planks.erase(planks.begin() + j);
                            j--;
                        }
                    }
                }
            }

        }
    }

    return planks;
}

void Planks::Draw(cv::Mat& image, std::vector<Planks::plank>& planks, std::vector<std::vector<cv::Point>>* contours) {
    const float halfPlankL = PLANK_L / 2.0f / PLANK_EPSI;
    const float halfPlankl = PLANK_l / 2.0f / PLANK_EPSI;
    for (Planks::plank& plank : planks) {
        const cv::Point2f perp = cv::Point2f(-plank.direction.y, plank.direction.x);
        const cv::Point2f p1 = plank.center + perp * halfPlankl + plank.direction * halfPlankL;
        const cv::Point2f p2 = plank.center - perp * halfPlankl + plank.direction * halfPlankL;
        const cv::Point2f p3 = plank.center - perp * halfPlankl - plank.direction * halfPlankL;
        const cv::Point2f p4 = plank.center + perp * halfPlankl - plank.direction * halfPlankL;
        cv::line(image, plank.center, plank.center + plank.direction * 100, cv::Scalar(0, 0, 255), 2);
        cv::circle(image, plank.center, 5, cv::Scalar(0, 0, 255), -1);
        cv::line(image, p1, p2, cv::Scalar(255, 0, 255), 2);
        cv::line(image, p2, p3, cv::Scalar(255, 0, 255), 2);
        cv::line(image, p3, p4, cv::Scalar(255, 0, 255), 2);
        cv::line(image, p4, p1, cv::Scalar(255, 0, 255), 2);
    }
    if (contours != nullptr) {  
        cv::drawContours(image, *contours, -1, cv::Scalar(255, 0, 0), 2);
    }
}

void Planks::Print(std::ostream& os, std::vector<Planks::plank>& planks) {
    for (const Planks::plank& plank : planks) {
        os << "Plank: " << plank.center << " " << plank.direction << std::endl;
    }
}
