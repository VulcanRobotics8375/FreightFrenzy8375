package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.robot.FreightFrenzyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp(name = "main")
public class Main extends OpModePipeline {

    FreightFrenzyConfig subsystems = new FreightFrenzyConfig();

    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void start() {
        super.start();

    }

    public void loop() {
        double startTimeMs = getRuntime()*1000.0;
        Robot.update();
        Pose2d robotPose = Robot.getRobotPose();

        subsystems.drivetrain.mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if(gamepad1.dpad_up) {
            subsystems.drivetrain.startOdometryLift();
        }

        subsystems.intake.run(gamepad2.left_bumper, gamepad2.right_bumper, gamepad2.dpad_up);

        subsystems.carousel.run(gamepad2.left_trigger > 0, gamepad2.dpad_right, gamepad2.right_trigger > 0);

        subsystems.lift.run(-gamepad2.left_stick_y, gamepad2.dpad_down, -gamepad2.right_stick_y, gamepad2.x, gamepad2.y, gamepad2.b, gamepad2.a);

        subsystems.cap.run(gamepad2.dpad_left);

//        telemetry.addData("imu angle", subsystems.drivetrain.getIMU().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//        telemetry.addData("left", Robot.getConfiguration().localizer.getRawWheelPositions().get(0));
//        telemetry.addData("right", Robot.getConfiguration().localizer.getRawWheelPositions().get(1));
//        telemetry.addData("strafe", Robot.getConfiguration().localizer.getRawWheelPositions().get(2));
//        telemetry.addData("x", robotPose.getX());
//        telemetry.addData("y", robotPose.getY());
//        telemetry.addData("heading", robotPose.getHeading());
        telemetry.update();

        double endTimeMs = getRuntime()*1000.0;
        maintainConstantLoopTime(60, endTimeMs - startTimeMs);
//        telemetry.addData("elapsed time ms", (getRuntime()*1000.0) - startTimeMs);

    }

}
