package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

public class AutoPaths extends OpModePipeline {

    public void init() {
        runMode = RobotRunMode.AUTONOMOUS;
        super.init();



    }

    public void start() {}

    @Override
    public void loop() {
        Robot.update();
        Pose2d robotPose = Robot.getRobotPose();
        Pose2d robotVel = Robot.getRobotVelocity();

        //everything should be async here,
        // also no multithreading unless you know what you are doing we will get problem with imu errors



    }
}
