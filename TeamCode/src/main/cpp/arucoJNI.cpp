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

        cv::cvtColor(mat, mat, cv::COLOR_RGB2GRAY);

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        std::vector<int> markerIds;

        cv::aruco::detectMarkers(mat, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        mat.release();

        jclass markerClass = env->FindClass("org/firstinspires/ftc/teamcode/vision/aruco/ArucoMarker");

//        if(markerClass == nullptr) {
//            return nullptr;
//        }

        jmethodID markerConstructor = env->GetMethodID(markerClass, "<init>", "([[DI)V");

        jobjectArray markerArray = env->NewObjectArray((jsize) markerCorners.size(), markerClass, nullptr);

        jclass doubleArrayClass = env->FindClass("[D");

//        if(doubleArrayClass == nullptr) {
//            return nullptr;
//        }

        //loop through detected markers

        for(int i = 0; i < markerCorners.size(); i++) {
            //add all 4 corners to 2d double array
            std::vector<cv::Point2f> marker = markerCorners[i];
            jobjectArray double2dArray = env->NewObjectArray((jsize) 4, doubleArrayClass, nullptr);
            for(int j = 0; j < 4; j++) {
                cv::Point2f corner = marker[j];
                double cornerArray[2] = {corner.x, corner.y};
                double *cornerArrayPtr = cornerArray;
                jdoubleArray doubleArray = env->NewDoubleArray((jsize) 2);
                env->SetDoubleArrayRegion(doubleArray, (jsize) 0, (jsize) 2, cornerArray);
                env->SetObjectArrayElement(double2dArray, j, doubleArray);
//                env->DeleteLocalRef(doubleArray);
            }
            //create marker object and add it to marker array
            int id = markerIds[i];
            jobject jmarkerObj = env->NewObject(markerClass, markerConstructor, double2dArray, (jint) id);
            env->SetObjectArrayElement(markerArray, i, jmarkerObj);
//            env->DeleteLocalRef(jmarkerObj);
//            env->DeleteLocalRef(double2dArray);
        }

        env->DeleteLocalRef(markerClass);
        env->DeleteLocalRef(doubleArrayClass);

        return markerArray;
}

extern "C"
JNIEXPORT jlong JNICALL
Java_org_firstinspires_ftc_teamcode_vision_aruco_ArucoMarker_nativeObj(JNIEnv *env, jobject thiz) {
    auto *marker = new ArucoMarker();
    return reinterpret_cast<jlong>(marker);
}
