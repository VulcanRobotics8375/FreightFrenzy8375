package org.firstinspires.ftc.teamcode.robotcorelib.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Robot {

    private static HardwareMap hardwareMap;
    private static Telemetry telemetry;

    private static Pose2d robotPose = new Pose2d();
    private static Pose2d robotVelocity = new Pose2d();

    private static RobotConfig config;

    public static void init(HardwareMap hwMap, Telemetry opModeTelemetry) {
        hardwareMap = hwMap;
        telemetry = opModeTelemetry;
        config = new RobotConfig();
        config.init();

        for (Subsystem sub : config.subsystems) {
            sub.setHardwareMap(hardwareMap);
            sub.setTelemetry(telemetry);
            sub.init();
        }

    }

    public static Pose2d getRobotPose() {
        return robotPose;
    }

    public static void setRobotPose(Pose2d robotPose) {
        Robot.robotPose = robotPose;
    }

    public static Pose2d getRobotVelocity() {
        return robotVelocity;
    }

    private static void setRobotVelocity(Pose2d robotVelocity) {
        Robot.robotVelocity = robotVelocity;
    }

    public static RobotConfig getConfiguration() {
        return config;
    }

    public static HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public static void updateGlobalPosition() {
        config.localizer.update();
        robotPose = config.localizer.getPoseEstimate();
        robotVelocity = config.localizer.getPoseVelocity();
    }

}
