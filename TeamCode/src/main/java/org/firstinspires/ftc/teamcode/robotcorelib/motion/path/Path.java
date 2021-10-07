package org.firstinspires.ftc.teamcode.robotcorelib.motion.path;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Point;

import java.util.ArrayList;

public class Path {

    private ArrayList<Point> points;
    private ArrayList<Double> lookaheads;
    private ArrayList<Double> turnSpeeds;
    private ArrayList<Double> speeds;
    private ArrayList<Double> angles;

    public Path(PathBuilder builder) {
        this.points = builder.points;
        this.lookaheads = builder.lookaheads;
        this.turnSpeeds = builder.turnSpeeds;
        this.speeds = builder.speeds;
        this.angles = builder.angles;
    }

    public void remove(int i) {
        points.remove(i);
        lookaheads.remove(i);
        turnSpeeds.remove(i);
        speeds.remove(i);
        angles.remove(i);
    }

    public int size() {
        return points.size();
    }

    public ArrayList<Point> getPoints() {
        return points;
    }

    public ArrayList<Double> getLookaheads() {
        return lookaheads;
    }

    public ArrayList<Double> getTurnSpeeds() {
        return turnSpeeds;
    }

    public ArrayList<Double> getSpeeds() {
        return speeds;
    }

    public ArrayList<Double> getAngles() {
        return angles;
    }

    public Point getPoint(int i) {
        return points.get(i);
    }

    public double getLookahead(int i) {
        return lookaheads.get(i);
    }

    public double getTurnSpeed(int i) {
        return turnSpeeds.get(i);
    }

    public double getSpeed(int i) {
        return speeds.get(i);
    }

    public double getAngle(int i) {
        return angles.get(i);
    }

}
