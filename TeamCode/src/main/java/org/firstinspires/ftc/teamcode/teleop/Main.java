package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcorelib.opmode.TeleOpPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "main")
public class Main extends TeleOpPipeline {

    public void init() {
        super.init();


    }

    public void start() {
        super.start();

    }

    public void loop() {
        Robot.updateGlobalPosition();
        Pose2d robotPose = Robot.getRobotPose();

//        subsystems.drivetrain.mechanumDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
        subsystems.intake.run(
                gamepad2.a, //intake
                gamepad2.b, //outtake
                gamepad2.left_bumper
        );
        subsystems.lift.run(-gamepad2.left_stick_y, gamepad2.y);
        subsystems.drivetrain.tankDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);
        subsystems.carousel.run(gamepad2.x);

//        telemetry.addData("left", Robot.getConfiguration().localizer.getWheelPositions().get(0));
//        telemetry.addData("right", Robot.getConfiguration().localizer.getWheelPositions().get(1));
//        telemetry.addData("strafe", Robot.getConfiguration().localizer.getWheelPositions().get(2));
//        telemetry.addData("x", robotPose.getX());
//        telemetry.addData("y", robotPose.getY());
//        telemetry.addData("heading", robotPose.getHeading());

        telemetry.update();

    }

}
