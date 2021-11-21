package org.firstinspires.ftc.teamcode.vision.aruco;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Point;
import java.util.ArrayList;

import java.util.List;

public class ArucoPipeline extends OpenCvPipeline {

    private Telemetry telemetry;

    public ArucoPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        ArucoMarker[] markers = detectArucoMarker(input.nativeObj);
//        telemetry.addLine("ran native method");
//        telemetry.update();
//        ArucoMarker detectedMarker = ArucoMarker.getMarkerById(markers, 50);
//
//        if(detectedMarker != null) {
//            ArrayList<Point> corners = detectedMarker.getCorners();
//            for(int i = 0; i < corners.size() - 1; i++) {
//                Point start = corners.get(i);
//                Point end = corners.get(i + 1);
//                Imgproc.line(input, start, end, new Scalar(0, 255, 0), 2, 8, 0);
//            }
//            Imgproc.line(input, corners.get(0), corners.get(3), new Scalar(0, 255, 0), 2, 8, 0);
//        }

        return input;
    }

    native ArucoMarker[] detectArucoMarker(long frame_ptr);

    static
    {
        System.loadLibrary("TeamCode");
    }
}
