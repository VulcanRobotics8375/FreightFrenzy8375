package org.firstinspires.ftc.teamcode.robotcorelib.motion.path;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Point;

import java.util.ArrayList;

public class PathBuilder {

    ArrayList<Point> points = new ArrayList<>();
    ArrayList<Double> lookaheads = new ArrayList<>();
    ArrayList<Double> turnSpeeds = new ArrayList<>();
    ArrayList<Double> speeds = new ArrayList<>();
    ArrayList<Double> angles = new ArrayList<>();
    double lookahead = 0, speed = 0, turnSpeed = 0, angle = 0;

    public PathBuilder() {}

    public PathBuilder moveTo(Point p) {
        points.add(p);
        lookaheads.add(lookahead);
        turnSpeeds.add(turnSpeed);
        speeds.add(speed);
        angles.add(angle);
        return this;
    }

    public PathBuilder setLookahead(double lookahead) {
        this.lookahead = lookahead;
        return this;
    }

    public PathBuilder setSpeed(double speed) {
        this.speed = speed;
        return this;
    }

    public PathBuilder setTurnSpeed(double turnSpeed) {
        this.turnSpeed = turnSpeed;
        return this;
    }

    public PathBuilder setTargetAngle(double angle) {
        this.angle = angle;
        return this;
    }

    public Path build() {
        return new Path(this);
    }

}
