#include <jni.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "aruco.hpp"

extern "C" JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_vision_aruco_ArucoPipeline_detectArucoMarker(JNIEnv *env, jobject type, jlong nativeMat) {

        auto* ptrMat = (cv::Mat*) nativeMat;
        cv::Mat mat = *ptrMat;

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        std::vector<int> markerIds;

        cv::aruco::detectMarkers(mat, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        cv::aruco::drawDetectedMarkers(mat, markerCorners, markerIds);

}
extern "C"
JNIEXPORT void JNICALL
Java_org_firstinspires_ftc_teamcode_vision_aruco_ArucoPipeline_detectMarkersNative(JNIEnv *env, jobject thiz, jlong frame_ptr, jobject markers_ptr) {
        auto* ptrMat = (cv::Mat*) frame_ptr;
        cv::Mat mat = *ptrMat;

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        std::vector<int> markerIds;

        cv::aruco::detectMarkers(mat, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

}


