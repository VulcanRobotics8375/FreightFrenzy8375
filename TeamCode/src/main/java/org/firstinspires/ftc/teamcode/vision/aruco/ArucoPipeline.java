package org.firstinspires.ftc.teamcode.vision.aruco;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class ArucoPipeline extends OpenCvPipeline {


    @Override
    public Mat processFrame(Mat input) {
        detectArucoMarker(input.nativeObj);
        return input;
    }

    native void detectArucoMarker(long frame_ptr);

    native void detectMarkersNative(long frame_ptr, double[] cornersX, double[] cornersY, int[] ids);

    static
    {
        System.loadLibrary("TeamCode");
    }
}
