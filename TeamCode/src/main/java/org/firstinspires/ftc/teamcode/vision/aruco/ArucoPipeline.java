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
    public boolean debug = true;
    public double boundingBoxBoundaryOne = 78, boundingBoxBoundaryTwo = 175;

    public volatile Point markerPos = new Point(0, 0);
    public volatile double markerX = 0;

    public ArucoPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        ArucoMarker[] markers = detectArucoMarker(input.nativeObj);
//        ArucoMarker detectedMarker = null;
        Point tempMarkerPose = new Point(0, 0);
        if(markers != null && markers.length > 0) {
            ArucoMarker marker = ArucoMarker.getMarkerById(markers, 42);


            if (marker != null) {
                tempMarkerPose = marker.getPosition();
                telemetry.update();

                //draw box
                if (debug) {
                    ArrayList<Point> corners = marker.getCorners();
                    for (int i = 0; i < corners.size() - 1; i++) {
                        Point start = corners.get(i);
                        Point end = corners.get(i + 1);
                        Imgproc.line(input, start, end, new Scalar(0, 255, 0), 2, 8, 0);
                    }
                    Imgproc.line(input, corners.get(0), corners.get(3), new Scalar(0, 255, 0), 2, 8, 0);

                    //label with id
                    Imgproc.putText(input, "x: " + marker.getPosition().x, markerPos, Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(0, 255, 0));
                }
            }


        }
        Imgproc.line(input, new Point(boundingBoxBoundaryOne, -200), new Point(boundingBoxBoundaryOne, 400), new Scalar(0, 0, 255), 2, 8, 0);
        Imgproc.line(input, new Point(boundingBoxBoundaryTwo, -200), new Point(boundingBoxBoundaryTwo, 400), new Scalar(255, 0, 0), 2, 8, 0);
        if(tempMarkerPose != null) {
            markerPos = tempMarkerPose;
            markerX = markerPos.x;
        }

        return input;
    }

    native ArucoMarker[] detectArucoMarker(long frame_ptr);

    static
    {
        System.loadLibrary("TeamCode");
    }
}
