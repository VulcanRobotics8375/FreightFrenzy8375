package org.firstinspires.ftc.teamcode.vision.aruco;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class ArucoPipeline extends OpenCvPipeline {


    @Override
    public Mat processFrame(Mat input) {
        ArucoMarker[] markers = detectArucoMarker(input.nativeObj);
        ArucoMarker detectedMarker = ArucoMarker.getMarkerById(markers, 50);
        
        Rect boundingRect = new Rect(detectedMarker.getCorners().get(0), detectedMarker.getCorners().get(2));
        Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 0));

        return input;
    }

    native ArucoMarker[] detectArucoMarker(long frame_ptr);

    static
    {
        System.loadLibrary("TeamCode");
    }
}
