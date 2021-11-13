package org.firstinspires.ftc.teamcode.vision.aruco;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class ArucoPipeline extends OpenCvPipeline {


    @Override
    public Mat processFrame(Mat input) {
        detectArucoMarker(input.nativeObj);
        return input;
    }

    native void detectArucoMarker(long frame_ptr);

    static
    {
        System.loadLibrary("TeamCode");
    }
}
