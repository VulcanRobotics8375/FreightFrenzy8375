package org.firstinspires.ftc.teamcode.robotcorelib.motion.followers;

import org.firstinspires.ftc.teamcode.robotcorelib.drive.DrivetrainInterface;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;

public abstract class Follower {

    DrivetrainInterface drivetrain;
    public Follower() {
        this.drivetrain = Robot.drivetrain;
    }

//    protected abstract void run();

}
