package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        Robot.update();

        subsystems.drivetrain.mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        subsystems.lift.test(-gamepad2.left_stick_y, gamepad2.right_stick_x, gamepad2.a, gamepad2.b);
        subsystems.lift.runTurretAndArm(
                gamepad2.x, //shared
                gamepad2.y, //alliance
                gamepad2.a, //home
                -gamepad2.left_stick_y, //lift manual
                gamepad2.right_stick_x, //turret manual
                gamepad2.b, //linkage
                gamepad2.right_bumper, //release
                gamepad2.right_trigger > 0, //linkage adjust out
                gamepad2.left_trigger >0, //linkage adjust in
                gamepad2.left_bumper //alliance side flip
        );

        subsystems.cap.run(gamepad2.dpad_left);
        subsystems.intake.run(gamepad2.dpad_down, gamepad2.x || gamepad2.y || gamepad2.left_bumper, subsystems.lift.isReset());

        if(gamepad1.dpad_up) {
            subsystems.drivetrain.startOdometryLift();
        }
        if(gamepad1.dpad_down) {
            subsystems.drivetrain.stopOdometryLift();
        }


        telemetry.update();

    }

}
