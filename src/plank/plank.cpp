#include "plank.hpp"

#define PLANK_DIAGONAL (float) sqrt(PLANK_L * PLANK_L + PLANK_l * PLANK_l)
#define PLANK_DIAGONAL_ANGLE (float) atan(PLANK_l / PLANK_L)
#define PLANK_DIAGONAL_ANGLE_COS (float) cos(PLANK_DIAGONAL_ANGLE)
#define PLANK_DIAGONAL_ANGLE_SIN (float) sin(PLANK_DIAGONAL_ANGLE)
#define MIN_PLANK_AREA PLANK_L * PLANK_l
#define MAX_PLANK_AREA PLANK_L * PLANK_l
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


void getFilteredImage(cv::Mat& base, cv::Mat& image, Arucos& arucos, cv::Mat& filtered) {
    cv::Point2f distortion;
    unsigned int i;

    /* difference between the images */
    cv::absdiff(base, image, filtered);
    cv::cvtColor(filtered, filtered, cv::COLOR_BGR2GRAY);
    cv::threshold(filtered, filtered, 50, 255, cv::THRESH_BINARY);
    
    /* hide the robots */
    for (i = Arucos::ROBOTS_MIN; i <= Arucos::ROBOTS_MAX; i++) {
        try {
            arucos.getDistortion((int) i, distortion);
            const cv::Point2f& pos = arucos[(int) i];
            cv::line(filtered, pos, pos + distortion * ROBOTS_RATIO, cv::Scalar(0), 2 * ROBOTS_RADIUS);
        } catch (const std::out_of_range& e) {
            /* robot not found */
        }
    }

    /* remove the alone pixels */
    cv::morphologyEx(filtered, filtered, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));

    /* fill the surfaces */
    cv::morphologyEx(filtered, filtered, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(41, 41)));
}

bool findPossiblePlank(Planks::plank& plank, const cv::Point2f& p1, const cv::Point2f& p2, const std::vector<cv::Point>& contour, unsigned int Nchecks, unsigned int threshold) {
    unsigned int missed, i;
    short int rectSens, diagSens;
    const float step = PLANK_DIAGONAL / Nchecks;

    for (rectSens = -1; rectSens <= 1; rectSens += 2) {
        missed = 0;

        for (diagSens = -1; diagSens <= 1; diagSens += 2) {
            const cv::Point2f line = diagSens * (p1 - p2);
            const cv::Point2f lineNorm = line / cv::norm(line);
            const cv::Point2f start = (diagSens < 0) ? p1 : p1 - lineNorm * PLANK_L;
            const cv::Point2f direction = cv::Point2f(
                lineNorm.x * PLANK_DIAGONAL_ANGLE_COS - lineNorm.y * diagSens * rectSens * PLANK_DIAGONAL_ANGLE_SIN,
                lineNorm.x * diagSens * rectSens * PLANK_DIAGONAL_ANGLE_SIN + lineNorm.y * PLANK_DIAGONAL_ANGLE_COS
            );

            i = 0;
            while (i < Nchecks && missed < threshold) {
                const cv::Point2f p = start + direction * (float) i * step;
                if (cv::pointPolygonTest(contour, p, false) < 0) {
                    missed++;
                    if (missed >= threshold) {
                        break;
                    }
                }
                i++;
            }

            if (missed >= threshold) {
                break;
            }
        }

        if (missed < threshold) {
            const cv::Point2f line = (p2 - p1) / cv::norm(p2 - p1);
            plank.center = p1 + line * PLANK_L / 2.0f + cv::Point2f(rectSens * line.y, -1 * rectSens * line.x) * PLANK_l / 2.0f;
            plank.direction = line;
            return true;
        }
    }
    return false;
}

// void Planks::RemoveInitPlanks(cv::Mat& image) {
//     for (const std::vector<cv::Point2f>& plank : initPlanksPos) {
//         const cv::Point2f& center = plank[0];
//         const cv::Point2f& halfPlank = plank[1] / cv::norm(plank[1]) * PLANK_L * INIT_PLANKS_RATIO / 2.0f;
//         cv::line(image, center, center + halfPlank, cv::Scalar(0), (int) (PLANK_l * INIT_PLANKS_RATIO));
//         cv::line(image, center, center - halfPlank, cv::Scalar(0), (int) (PLANK_l * INIT_PLANKS_RATIO));
//     }
// }

std::vector<Planks::plank> Planks::Get(cv::Mat& base, cv::Mat& image, Arucos& arucos, std::vector<std::vector<cv::Point>>* contours) {
    cv::Mat filtered, canny_output;
    std::vector<Planks::plank> planks;
    Planks::plank plank;
    std::vector<std::vector<cv::Point>> conts;
    std::vector<cv::Point2f> robotsPos;
    unsigned int i, j;

    getFilteredImage(base, image, arucos, filtered);

    cv::Canny(filtered, canny_output, 250, 250);
    cv::findContours(canny_output, conts, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for (i = Arucos::ROBOTS_MIN; i <= Arucos::ROBOTS_MAX; i++) {
        try {
            const cv::Point2f arucoCenter = arucos[(int) i];
            robotsPos.push_back(arucoCenter);
        } catch (const std::out_of_range& e) {
            /* robot not found */
        }
    }

    for (std::vector<cv::Point>& contour : conts) {
        /* approximate the contour to a polynom */
        cv::approxPolyDP(contour, contour, 30, true);

        const double area = cv::contourArea(contour);
        if (area >= MIN_PLANK_AREA) {
            for (i = 0; i < contour.size(); i++) {
                const cv::Point2f p1 = contour[i];
                const cv::Point2f p2 = contour[(i + 1) % contour.size()];

                if (findPossiblePlank(plank, p1, p2, contour, N_CONTROL_POINTS, N_CONTROL_POINTS_THRESHOLD)) {
                    planks.push_back(plank);

                    if (contours != nullptr) {
                        contours->push_back(contour);
                    }
                } else {
                    if (findPossiblePlank(plank, p2, p1, contour, N_CONTROL_POINTS, N_CONTROL_POINTS_THRESHOLD)) {
                        planks.push_back(plank);

                        if (contours != nullptr) {
                            contours->push_back(contour);
                        }
                    }
                }
            }
        }
    }

    for (i = 0; i < planks.size(); i++) {
        Planks::plank& plank = planks[i];

        j = 0;
        while (j < robotsPos.size() && cv::norm(robotsPos[j] - plank.center) >= MIN_DST_ROBOTS_PLANK) { /* TODO test */
            j++;
        }
        if (j < robotsPos.size()) {
            planks.erase(planks.begin() + i);
            i--;
        } else {

            for (j = i + 1; j < planks.size(); j++) {
                const Planks::plank& p = planks[j];
                if (cv::norm(p.center - plank.center) < PLANK_l) {
                    if  (cv::norm(p.direction - plank.direction) < 0.1) {
                        plank.center = (plank.center + p.center) / 2;
                        plank.direction = (plank.direction + p.direction) / 2;
                        planks.erase(planks.begin() + j);
                        j--;
                    } else {
                        if (cv::norm(p.direction + plank.direction) < 0.1) {
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
        std::cout << "Plank at " << plank.center << " with direction " << plank.direction << std::endl;
    }
    if (contours != nullptr) {  
        cv::drawContours(image, *contours, -1, cv::Scalar(0, 255, 0), 2);
    }
}
