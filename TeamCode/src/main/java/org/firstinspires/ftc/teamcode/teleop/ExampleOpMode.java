package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.LVMaxbotixEZ4;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.LVMaxbotixTest;

@TeleOp(group = "example", name = "range sensor test")
public class ExampleOpMode extends OpModePipeline {
    LVMaxbotixTest rangeSensor;

    public void init() {
        runMode = RobotRunMode.TELEOP;
        super.init();
        rangeSensor = new LVMaxbotixTest(hardwareMap, "sensor_1");

    }

    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        Robot.update();
        Pose2d robotPose = Robot.getRobotPose();
        telemetry.addData("range sensor", rangeSensor.sensor.getVoltage());
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());
        telemetry.update();
    }
}
