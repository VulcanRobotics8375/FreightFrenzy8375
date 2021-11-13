//
// Created by Nico Paoli on 11/13/21.
//

#ifndef FREIGHTFRENZY8375_ARUCOMARKER_H
#define FREIGHTFRENZY8375_ARUCOMARKER_H

#include <vector>
#include <opencv2/core.hpp>
#include "include/aruco.hpp"

class ArucoMarker {
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;



};


#endif //FREIGHTFRENZY8375_ARUCOMARKER_H
