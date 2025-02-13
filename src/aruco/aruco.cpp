#include "aruco.hpp"

#define ROBOTS_RATIO_FIRST (1.0f / (CAMERA_Z / ROBOTS_ARUCO_Z - 1.0f))
#define ROBOTS_RATIO_SECOND (1.0f - ROBOTS_ARUCO_Z / CAMERA_Z)

const cv::aruco::Dictionary Arucos::dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
const cv::aruco::DetectorParameters Arucos::detectorParams = cv::aruco::DetectorParameters();
const cv::aruco::ArucoDetector Arucos::detector = cv::aruco::ArucoDetector(dictionary, detectorParams);
const std::vector<cv::Point2f> Arucos::dst = {
    cv::Point2f(2400, 600),
    cv::Point2f(600, 600),
    cv::Point2f(600, 1400),
    cv::Point2f(2400, 1400)
};


Arucos::Arucos() {
    cornersOutdated = true;
}

void Arucos::nextFrame(cv::Mat& image) {
    std::vector<int> toErase;
    cv::Point2f last;
    unsigned int i, j;

    /* detect arucos */
    detector.detectMarkers(image, corners, ids, rejectedCandidates);

    /* udpate or add the arucos */
    for (i = 0; i < ids.size(); i++) {
        const int id = ids[i];

        /* get the center of the aruco */
        cv::Point2f center(0, 0);
        for (cv::Point2f& corner : corners[i]) {
            center += corner;
        }
        center /= 4.0f;

        if (elements.find(id) == elements.end()) {
            /* First time detecting this aruco. Its raw
            position is set while the other positions
            are reseted */
            elements[id].raw = center;
            elements[id].aruco = cv::Point2f(-1.0f, -1.0f);
            for (j = 0; j < ARUCO_POS_MEMORY; j++) {
                elements[id].real[j] = cv::Point2f(-1.0f, -1.0f);
            }
            elements[id].notFound = 0;
        } else {
            /* The aruco has already been find. Its raw
            position is updated while the aruco position as
            well as the last real position and notFound
            attribute are resetted. The formers real
            positions are moved in the array */
            elements[id].raw = center;
            elements[id].aruco = cv::Point2f(-1.0f, -1.0f);
            for (j = ARUCO_POS_MEMORY - 1; j > 0; j--) {
                elements[id].real[j] = elements[id].real[j - 1];
            }
            elements[id].real[0] = cv::Point2f(-1.0f, -1.0f);
            elements[id].notFound = 0;
        }
    }

    /* for each element we detected so far in the frames */
    for (std::pair<const int, Arucos::element>& elem : elements) {

        /* check if the aruco is not found */
        if (std::find(ids.begin(), ids.end(), elem.first) == ids.end()) {
            elem.second.notFound++;
            if (elem.second.notFound > ARUCO_NOTFOUND_THRESHOLD) {
                /* the aruco has not been find for a long time,
                it is removed */
                toErase.push_back(elem.first);
            } else {
                /* the aruco has not been find for a short time,
                we resetted its raw and aruco positions */
                elem.second.raw = cv::Point2f(-1.0f, -1.0f);
                elem.second.aruco = cv::Point2f(-1.0f, -1.0f);

                /* find the last known real position */
                i = ARUCO_POS_MEMORY - 1;
                last = elem.second.real[i];
                i--;
                while (i > 0 && last.x < 0) {
                    last = elem.second.real[i];
                    i--;
                }
                if (last.x < 0) {
                    /* the formers positions are moved in the array
                    and the first position is guessed from the previous
                    ones */
                    if (i < ARUCO_POS_MEMORY - 1) {
                        i++;
                    }
                    for (j = i; j > 0; j--) {
                        elem.second.real[j] = elem.second.real[j - 1];
                    }
                    elem.second.real[0] += (elem.second.real[0] - last) / static_cast<float>(i);
                } else {
                    /* we do not know where the element is and we only know its previous position: assuming it did not move */
                }
            }
        }
    }
    for (int id : toErase) {
        elements.erase(id);
    }
}

cv::Point2f& Arucos::operator[](int id) {
    return this->getPosition(id, true, false);
}

cv::Point2f& Arucos::getPosition(int id, bool projected, bool aruco) {
    if (elements.find(id) == elements.end()) {
        throw std::out_of_range("Aruco " + std::to_string(id) + " does not exist");
    }
    if (!projected) {
        if (elements[id].notFound > 0) {
            throw std::runtime_error("Aruco " + std::to_string(id) + " not found"); 
        }
        return elements[id].raw;
    }
    if (aruco) {
        if (elements[id].aruco.x < 0) {
            throw std::runtime_error("Aruco " + std::to_string(id) + " is outdated");
        }
        return elements[id].aruco;
    }
    if (elements[id].real[0].x < 0) {
        throw std::runtime_error("Aruco " + std::to_string(id) + " is outdated");
    }
    return elements[id].real[0];
}

void Arucos::warp(cv::Mat& input, cv::Mat& output, bool usePreviousMatrix, bool forceUpdateMatrix) {
    std::vector<cv::Point2f> src(4), centers;
    unsigned int i;

    /* get the transformation matrix */
    if (!usePreviousMatrix) {
        try {
            src[0] = this->getPosition(Arucos::CENTER_TOP_LEFT, false);
            src[1] = this->getPosition(Arucos::CENTER_TOP_RIGHT, false);
            src[2] = this->getPosition(Arucos::CENTER_BOTTOM_LEFT, false);
            src[3] = this->getPosition(Arucos::CENTER_BOTTOM_RIGHT, false);
        } catch (const std::exception& e) {
            if (forceUpdateMatrix || transformMatrix.empty()) {
                throw std::runtime_error("Not all center markers found!" + std::string(e.what()));
            } else {
                src.clear();
            }
        }

        if (!src.empty()) {
            transformMatrix = cv::getPerspectiveTransform(src, dst);
        }
    }

    /* warp the image */
    cv::warpPerspective(input, output, transformMatrix, cv::Size(3000, 2000));

    /* update the elements with their warpped position */
    for (const std::pair<const int, Arucos::element>& elem : elements) {
        if (elem.second.notFound == 0) {
            centers.push_back(elem.second.raw);
        }
    }
    cv::perspectiveTransform(centers, centers, transformMatrix);
    i = 0;
    for (std::pair<const int, Arucos::element>& elem : elements) {
        if (elem.second.notFound == 0) {
            elem.second.aruco = centers[i];
            if (elem.first < Arucos::ROBOTS_MIN || elem.first > Arucos::ROBOTS_MAX) {
                /* if it is not a robot, the aruco is assumed
                to be close to the table and so the real position
                is the aruco's one */
                elem.second.real[0] = centers[i];
            } else {
                /* if it is a robot, the real position is the
                deduced from the aruco's one (just the Thales
                theorem) */
                elem.second.real[0] = CAMERA_POS + (centers[i] - CAMERA_POS) * ROBOTS_RATIO_SECOND;
            }
            i++;
        }
    }

    cornersOutdated = true;
}

void Arucos::getDistortion(int id, cv::Point2f& distortion) {
    if (id < Arucos::ROBOTS_MIN || id > Arucos::ROBOTS_MAX) {
        throw std::out_of_range("Aruco " + std::to_string(id) + " is not a robot");
    }
    // distortion = this->getPosition(id, true, true) - this->getPosition(id, true, false);
    /* the distortion is deduced from the real position using
    the Thales theorem */
    distortion = ((*this)[id] - CAMERA_POS) * ROBOTS_RATIO_FIRST;
}

void Arucos::draw(cv::Mat& input) {
    int i;

    if (cornersOutdated) {
        for (const std::vector<cv::Point2f>& c : corners) {
            cv::perspectiveTransform(c, c, transformMatrix);
        }
        cornersOutdated = false;
    }
    cv::aruco::drawDetectedMarkers(input, corners, ids);

    for (i = Arucos::ROBOTS_MIN; i <= Arucos::ROBOTS_MAX; i++) {
        try {
            cv::circle(input, (*this)[i], 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(input, (*this)[i], 150, cv::Scalar(0, 255, 0), 2);
        } catch (const std::exception& e) {
            /* robot not found */
            UNUSED(e);
        }
    }
}

void Arucos::print(std::ostream& os) {
    for (const std::pair<const int, Arucos::element>& elem : elements) {
        os << "Aruco " << elem.first << ": raw(" << elem.second.raw << ") aruco(" << elem.second.aruco << ") real(" << elem.second.real[0] << ") notFound(" << elem.second.notFound << ")" << std::endl;
    }
}
