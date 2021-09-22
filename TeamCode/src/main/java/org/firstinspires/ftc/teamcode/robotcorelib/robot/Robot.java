package org.firstinspires.ftc.teamcode.robotcorelib.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

import java.util.List;

public class Robot {

    private static HardwareMap hardwareMap;
    private static Telemetry telemetry;

    private static Pose2d robotPose = new Pose2d();
    private static Pose2d robotVelocity = new Pose2d();

    private static List<LynxModule> hubs;
    //default is manual since the SDK is set up to do so
    private static LynxModule.BulkCachingMode bulkCachingMode = LynxModule.BulkCachingMode.MANUAL;

    private static RobotConfig config;

    public static void init(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        config = new RobotConfig();
        config.init();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(bulkCachingMode);
        }

        for (Subsystem sub : config.subsystems) {
            sub.setHardwareMap(hardwareMap);
            sub.setTelemetry(telemetry);
            sub.assignGamePads(opMode.gamepad1, opMode.gamepad2);
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

    public static void setBulkCachingMode(LynxModule.BulkCachingMode bulkCachingMode) {
        Robot.bulkCachingMode = bulkCachingMode;
    }

    public static LynxModule.BulkCachingMode getBulkCachingMode() {
        return bulkCachingMode;
    }

    public static void updateGlobalPosition() {
        config.localizer.update();
        robotPose = config.localizer.getPoseEstimate();
        robotVelocity = config.localizer.getPoseVelocity();
    }

    public static void clearBulkCache() {
        if(bulkCachingMode == LynxModule.BulkCachingMode.MANUAL) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
        }
    }

    public static void update() {
        clearBulkCache();
        updateGlobalPosition();
    }

}
