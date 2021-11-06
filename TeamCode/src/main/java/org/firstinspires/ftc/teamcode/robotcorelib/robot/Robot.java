package org.firstinspires.ftc.teamcode.robotcorelib.robot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcorelib.drive.DrivetrainImpl;
import org.firstinspires.ftc.teamcode.robotcorelib.drive.localization.Localizer;
import org.firstinspires.ftc.teamcode.robotcorelib.util.ErrorHandler;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

import java.util.List;

public class Robot {

    private static HardwareMap hardwareMap;
    private static Telemetry telemetry;
    private static ErrorHandler errorHandler;

    private static Localizer localizer;

    private static Pose2d robotPose = new Pose2d();
    private static Pose2d robotVelocity = new Pose2d();

    private static List<LynxModule> hubs;
    //default is manual since robotcorelib is set up to do so
    private static LynxModule.BulkCachingMode bulkCachingMode = LynxModule.BulkCachingMode.MANUAL;

    private static RobotConfig config;

    public static RobotRunMode runMode;
    public static DrivetrainImpl drivetrain;

    public static void init(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        telemetry.addLine("loading configuration");
        telemetry.update();
        errorHandler = new ErrorHandler(telemetry);
        config = new RobotConfig();
        config.init();
        localizer = config.localizer;

        telemetry.addLine("updating REV hub cache mode");
        telemetry.update();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(bulkCachingMode);
        }

        for (Subsystem sub : config.subsystems) {
            sub.setHardwareMap(hardwareMap);
            sub.setTelemetry(telemetry);
            sub.assignGamePads(opMode.gamepad1, opMode.gamepad2);
            if(sub instanceof DrivetrainImpl) {
                drivetrain = (DrivetrainImpl) sub; // cast drivetrain to drivetrainImpl, this is for backend controller stuff.
            }
            sub.init();
            telemetry.addData("initializing subsystem", sub.toString());
            telemetry.update();
        }
        telemetry.addLine("init complete");
        //no telemetry.update() here because that happens in OpModePipeline

    }

    public static Pose2d getRobotPose() {
        return robotPose;
    }

    public static void setRobotPose(Pose2d robotPose) {
        Robot.localizer.setPose(robotPose);
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
        localizer.update();
        robotPose = localizer.getPose();
        robotVelocity = localizer.getVelocity();
    }

    public static void clearBulkCache() {
        if(bulkCachingMode == LynxModule.BulkCachingMode.MANUAL) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
        }
    }

    public static void addErrorMessage(String message) {
        errorHandler.addMessage(message);
    }

    public static void update() {
        clearBulkCache();
        updateGlobalPosition();
        errorHandler.update(false);
        telemetry.update();
    }

    public static void setRunMode(RobotRunMode runMode) {
        Robot.runMode = runMode;
    }

}
