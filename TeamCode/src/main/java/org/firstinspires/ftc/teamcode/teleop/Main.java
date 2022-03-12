package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.FreightFrenzyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

import java.util.List;

@TeleOp(name = "main")
public class Main extends OpModePipeline {

    FreightFrenzyConfig subsystems = new FreightFrenzyConfig();

    private boolean dpad_up = false;

    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void start() {
        super.start();

    }

    private TeleOpState teleOpState = TeleOpState.MAIN;
    public void loop() {
        Robot.update();

        telemetry.addData("imu angle", subsystems.drivetrain.getIMU().getAngularOrientation().firstAngle);
        subsystems.drivetrain.mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        telemetry.addData("left wheel", subsystems.localizer.getWheelPositions().get(0));
        telemetry.addData("right wheel", subsystems.localizer.getWheelPositions().get(1));
        telemetry.addData("center wheel", subsystems.localizer.getWheelPositions().get(2));

        TeleOpState prevTeleOpState = teleOpState;
        if(gamepad2.dpad_up && !this.dpad_up) {
            this.dpad_up = true;
            teleOpState = (teleOpState == TeleOpState.MAIN) ? TeleOpState.NO_LIMITS : TeleOpState.MAIN;
        } else if(!gamepad2.dpad_up && this.dpad_up) {
            this.dpad_up = false;
        }

        switch (teleOpState) {
            case MAIN:
                if(prevTeleOpState != teleOpState) {
                    subsystems.lift.reset();
                }
                subsystems.lift.runTurretAndArm(
                        gamepad2.x, //shared
                        gamepad2.y, //alliance
                        gamepad2.a, //home
                        -gamepad2.left_stick_y, //lift manual
                        gamepad2.right_stick_x, //turret manual
                        gamepad2.b, //linkage
                        gamepad2.right_bumper, //release
                        gamepad2.right_trigger, //linkage adjust out
                        gamepad2.left_trigger, //linkage adjust in
                        gamepad2.dpad_left //alliance side flip
                );
                break;
            case NO_LIMITS:
                subsystems.lift.runNoLimits(
                        gamepad2.left_stick_y, //lift manual
                        gamepad2.right_stick_x //turret manual
                );
                break;
        }

        subsystems.carousel.run(gamepad2.dpad_right);

        subsystems.cap.run(gamepad1.left_trigger, gamepad1.right_trigger);
        subsystems.intake.run(gamepad2.dpad_down, gamepad2.x || gamepad2.y || gamepad2.left_bumper, subsystems.lift.isReset());

        if(gamepad1.dpad_up) {
            subsystems.drivetrain.startOdometryLift();
        }
        if(gamepad1.dpad_down) {
            subsystems.drivetrain.stopOdometryLift();
        }


        telemetry.update();

    }

    public enum TeleOpState {
        MAIN,
        NO_LIMITS
    }

}
