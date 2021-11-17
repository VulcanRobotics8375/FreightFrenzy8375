//
// Created by Nico Paoli on 11/13/21.
//

#ifndef FREIGHTFRENZY8375_ARUCOMARKER_H
#define FREIGHTFRENZY8375_ARUCOMARKER_H

#include <vector>
#include <opencv2/core.hpp>
#include "include/aruco.hpp"

class ArucoMarker {
private:
    std::vector<cv::Point2f> corners;
    int id;

public:

    ArucoMarker() {};

    ArucoMarker(std::vector<cv::Point2f> corners, int id) : corners(corners), id(id) {};

    std::vector<cv::Point2f> getCorners() {
        return corners;
    }

    int getId() {
        return id;
    }

    void setCorners(std::vector<cv::Point2f> corners_new) {
        corners = corners_new;
    }

    void setId(int id_new) {
        id = id_new;
    }

};


#endif //FREIGHTFRENZY8375_ARUCOMARKER_H
