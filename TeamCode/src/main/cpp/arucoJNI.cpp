#include <jni.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include "aruco.hpp"
#include "ArucoMarker.h"

extern "C" JNIEXPORT jobjectArray JNICALL
Java_org_firstinspires_ftc_teamcode_vision_aruco_ArucoPipeline_detectArucoMarker(JNIEnv *env, jobject type, jlong nativeMat) {

        auto* ptrMat = (cv::Mat*) nativeMat;
        cv::Mat mat = *ptrMat;

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        std::vector<int> markerIds;

        cv::aruco::detectMarkers(mat, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        jclass markerClass = env->FindClass("org/firstinspires/ftc/teamcode/vision/aruco/ArucoMarker");
        jmethodID markerConstructor = env->GetMethodID(markerClass, "<init>", "([[DI)V");

        jobjectArray markerArray = env->NewObjectArray(markerCorners.size(), markerClass, NULL);

        jclass doubleArrayClass = env->FindClass("[D");

        //loop through detected markers
        for(int i = 0; i < markerCorners.size(); i++) {
            //add all 4 corners to 2d double array
            std::vector<cv::Point2f> marker = markerCorners[i];
            jobjectArray double2dArray = env->NewObjectArray(4, doubleArrayClass, NULL);
            for(int j = 0; j < 4; j++) {
                cv::Point2f corner = marker[j];
                double cornerArray[] = {corner.x, corner.y};
                jdoubleArray doubleArray = env->NewDoubleArray(2);
                env->SetDoubleArrayRegion(doubleArray, 0, 2, cornerArray);
                env->SetObjectArrayElement(double2dArray, i, doubleArray);
                env->DeleteLocalRef(doubleArray);
            }
            int id = markerIds[i];
            jobject jmarkerObj = env->NewObject(markerClass, markerConstructor, double2dArray, id);
            env->SetObjectArrayElement(markerArray, i, jmarkerObj);
            env->DeleteLocalRef(jmarkerObj);
        }

        return markerArray;


}

extern "C"
JNIEXPORT jlong JNICALL
Java_org_firstinspires_ftc_teamcode_vision_aruco_ArucoMarker_nativeObj(JNIEnv *env, jobject thiz) {
    auto *marker = new ArucoMarker();
    return reinterpret_cast<jlong>(marker);
}
