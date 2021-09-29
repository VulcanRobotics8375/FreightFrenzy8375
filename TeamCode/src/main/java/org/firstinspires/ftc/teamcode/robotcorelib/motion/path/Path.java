package org.firstinspires.ftc.teamcode.robotcorelib.motion.path;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Point;

import java.util.ArrayList;

public class Path {

    ArrayList<Point> points;
    ArrayList<Double> lookaheads;
    ArrayList<Double> turnSpeeds;
    ArrayList<Double> speeds;
    ArrayList<Double> angles;

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



}
