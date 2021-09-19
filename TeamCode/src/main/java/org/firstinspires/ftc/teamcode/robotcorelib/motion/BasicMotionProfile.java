package org.firstinspires.ftc.teamcode.robotcorelib.motion;

public abstract class BasicMotionProfile {

    private double maxAccel;
    private double maxVel;
    

    public abstract void get(double t);

}
