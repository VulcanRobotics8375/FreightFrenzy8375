package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.FreightFrenzyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.kinematics.DriveKinematics;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp
public class FieldCentricTest extends OpModePipeline {

    private final FreightFrenzyConfig subsystems = new FreightFrenzyConfig();

    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();

    }


    @Override
    public void loop() {
        Robot.update();
        Pose2d robotPose = Robot.getRobotPose();
        subsystems.drivetrain.setPowers(DriveKinematics.mecanumFieldVelocityToWheelVelocities(robotPose, new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x)));
//        subsystems.drivetrain.mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        subsystems.drivetrain.setPowers(-gamepad1.left_stick_y, -gamepad1.right_stick_y, -gamepad2.left_stick_y, -gamepad2.right_stick_y);


//        telemetry.addData("left", Robot.getConfiguration().localizer.getRawWheelPositions().get(0));
//        telemetry.addData("right", Robot.getConfiguration().localizer.getRawWheelPositions().get(1));
//        telemetry.addData("strafe", Robot.getConfiguration().localizer.getRawWheelPositions().get(2));
        telemetry.addData("x", robotPose.getX());
        telemetry.addData("y", robotPose.getY());
        telemetry.addData("heading", robotPose.getHeading());

    }
}
