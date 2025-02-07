#include "plank.hpp"

#define PLANK_EPSI 0.9
#define PLANK_L 400 * PLANK_EPSI
#define PLANK_l 100 * PLANK_EPSI
#define PLANK_DIAGONAL (float) sqrt(PLANK_L * PLANK_L + PLANK_l * PLANK_l)
#define PLANK_DIAGONAL_ANGLE (float) atan(PLANK_l / PLANK_L)
#define PLANK_DIAGONAL_ANGLE_COS (float) cos(PLANK_DIAGONAL_ANGLE)
#define PLANK_DIAGONAL_ANGLE_SIN (float) sin(PLANK_DIAGONAL_ANGLE)
#define N_CONTROL_POINTS 18
#define N_CONTROL_POINTS_THRESHOLD 2
#define MIN_PLANK_AREA PLANK_L * PLANK_l
#define MAX_PLANK_AREA PLANK_L * PLANK_l
#define MAX_DST_ROBOTS_PLANK 100


void getFilteredImage(cv::Mat& base, cv::Mat& image, cv::Mat& filtered) {
    /* difference between the images */
    cv::absdiff(base, image, filtered);
    cv::cvtColor(filtered, filtered, cv::COLOR_BGR2GRAY);
    cv::threshold(filtered, filtered, 50, 255, cv::THRESH_BINARY);

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

std::vector<Planks::plank> Planks::Get(cv::Mat& base, cv::Mat& image, Arucos& arucos, std::vector<std::vector<cv::Point>>* contours) {
    cv::Mat filtered, canny_output;
    std::vector<Planks::plank> planks;
    Planks::plank plank;
    std::vector<std::vector<cv::Point>> conts;
    std::vector<cv::Point2f> robotsPos;
    // bool isRobot;
    unsigned int i, j;

    getFilteredImage(base, image, filtered);

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

    return planks;
}

void Planks::Draw(cv::Mat& image, std::vector<Planks::plank>& planks, std::vector<std::vector<cv::Point>>* contours) {
    for (Planks::plank& plank : planks) {
        cv::line(image, plank.center, plank.center + plank.direction * 100, cv::Scalar(0, 0, 255), 2);
        cv::circle(image, plank.center, 5, cv::Scalar(0, 0, 255), -1);
        std::cout << "Plank at " << plank.center << " with direction " << plank.direction << std::endl;
    }
    if (contours != nullptr) {  
        cv::drawContours(image, *contours, -1, cv::Scalar(0, 255, 0), 2);
    }
}
