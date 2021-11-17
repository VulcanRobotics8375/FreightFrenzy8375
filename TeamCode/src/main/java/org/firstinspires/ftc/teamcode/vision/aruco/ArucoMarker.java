package org.firstinspires.ftc.teamcode.vision.aruco;

import org.opencv.core.MatOfPoint2f;

public class ArucoMarker {

    public long nativeObj;
    private double[][] corners;
    private int id;

    public ArucoMarker(double[][] corners, int id) {
        this.corners = corners;
        this.id = id;
        nativeObj = nativeObj();
    }

    private native long nativeObj();

}
