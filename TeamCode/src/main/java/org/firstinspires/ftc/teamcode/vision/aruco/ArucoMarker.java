package org.firstinspires.ftc.teamcode.vision.aruco;

import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import java.util.ArrayList;

public class ArucoMarker {

    public long nativeObj;
    private final double[][] corners;
    public int id;

    public ArucoMarker(double[][] corners, int id) {
        this.corners = corners;
        this.id = id;
        nativeObj = nativeObj();
    }

    public ArrayList<Point> getCorners() {
        ArrayList<Point> points = new ArrayList<>();
        for (double[] corner : corners) {
            points.add(new Point(corner[0], corner[1]));
        }

        return points;
    }

    public Point getPosition() {
        ArrayList<Point> corners = getCorners();
        //get opposite corners
        Point p1 = corners.get(0);
        Point p2 = corners.get(2);

        return new Point((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0);
    }

    public static ArucoMarker getMarkerById(ArucoMarker[] markers, int id) {
        for(ArucoMarker marker : markers) {
            if(marker.id == id) {
                return marker;
            }
        }
        return null;
    }

    private native long nativeObj();

}
