package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp(name = "main")
public class Main extends OpModePipeline {

    public void init() {
        runMode = RobotRunMode.TELEOP;
        super.init();


    }

    public void start() {
        super.start();

    }

    public void loop() {
        Robot.update();
        Pose2d robotPose = Robot.getRobotPose();

        subsystems.drivetrain.mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        subsystems.intake.run(gamepad2.a);
//        telemetry.addData("left", Robot.getConfiguration().localizer.getWheelPositions().get(0));
//        telemetry.addData("right", Robot.getConfiguration().localizer.getWheelPositions().get(1));
//        telemetry.addData("strafe", Robot.getConfiguration().localizer.getWheelPositions().get(2));
//        telemetry.addData("x", robotPose.getX());
//        telemetry.addData("y", robotPose.getY());
//        telemetry.addData("heading", robotPose.getHeading());

    }

}
