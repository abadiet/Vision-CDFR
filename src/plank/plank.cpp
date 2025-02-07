#include "plank.hpp"

#define PLANK_EPSI 0.9
#define PLANK_L 400 * PLANK_EPSI
#define PLANK_l 100 * PLANK_EPSI
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

void getFullLine(cv::Point2f& p1, cv::Point2f& p2, std::vector<cv::Point>& contour, float step) {
    const cv::Point2f line = p2 - p1;
    const cv::Point2f direction = line / cv::norm(line);
    float k = 1.0f;
    while (cv::pointPolygonTest(contour, p2 + direction * k * step, false) >= 0) {
        k++;
    }
    p2 = p2 + direction * (k - 1) * step;
    float j = 1.0f;
    while (cv::pointPolygonTest(contour, p1 - direction * j * step, false) >= 0) {
        j++;
    }
    p1 = p1 - direction * (j - 1) * step;
}

std::vector<Planks::plank> Planks::Get(cv::Mat& base, cv::Mat& image, Arucos& arucos, std::vector<std::vector<cv::Point>>* contours) {
    cv::Mat filtered, canny_output;
    std::vector<Planks::plank> planks;
    Planks::plank plank;
    std::vector<std::vector<cv::Point>> conts, newconts;
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
                const float step = 10.0f;

                cv::Point2f p1 = contour[i];
                cv::Point2f p2 = contour[(i + 1) % contour.size()];
                getFullLine(p1, p2, contour, step);
                const cv::Point2f fullLine = p2 - p1;
                const double lenght = cv::norm(fullLine);
                if (lenght >= PLANK_L) {
                    const cv::Point2f centerLine = (p1 + p2) / 2;

                    for (short int sens = 1; sens >= -1; sens -= 2) {
                        const cv::Point2f Plank_l = (cv::Point2f(-1 * sens * fullLine.y, sens * fullLine.x) / lenght) * PLANK_l;
                        const cv::Point2f centerOpposite = centerLine + Plank_l;
                        if (cv::pointPolygonTest(contour, centerOpposite, false) > 0) {
                            plank.center = centerLine + Plank_l / 2.0;
                            plank.direction = fullLine / lenght;
                            planks.push_back(plank);

                            if (contours != nullptr) {
                                contours->push_back(contour);
                            }

                            // cv::line(image, p1, p2, cv::Scalar(255, 0, 0), 2);
                            // Planks::Draw(image, planks);
                            // showImage("Oui", image);

                            break;
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
            if (cv::norm(p.center - plank.center) < PLANK_l / 2.0f) {
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
        std::cout << "Plank at " << plank.center << " with direction " << plank.direction << std::endl;
    }
    if (contours != nullptr) {  
        cv::drawContours(image, *contours, -1, cv::Scalar(0, 255, 0), 2);
    }
}
