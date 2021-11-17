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
        
        if(detectedMarker != null) {
            ArrayList<Point> corners = detectedMarker.getCorners();
            for(int i = 0; i < corners.size() - 1; i++) {
                Point start = corners.get(i);
                Point end = corners.get(i + 1);
                Imgproc.line(input, start, end, new Scalar(0, 255, 0), 2, 8, 0);
            }
            Imgproc.line(input, corners.get(0), corners.get(3), new Scalar(0, 255, 0), 2, 8, 0);
        }

        return input;
    }

    native ArucoMarker[] detectArucoMarker(long frame_ptr);

    static
    {
        System.loadLibrary("TeamCode");
    }
}
