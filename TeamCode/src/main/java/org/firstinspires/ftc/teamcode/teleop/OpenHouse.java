package org.firstinspires.ftc.teamcode.teleop;

import org.firstinspires.ftc.teamcode.robot.FreightFrenzyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

public class OpenHouse extends OpModePipeline {
    FreightFrenzyConfig subsystems = new FreightFrenzyConfig();

    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.TELEOP;
        super.init();
    }

    public void start() {
        super.start();

    }

    private Main.TeleOpState teleOpState = Main.TeleOpState.MAIN;
    public void loop() {
        Robot.update();

        subsystems.drivetrain.mechanumDrive(-gamepad1.left_stick_y * .5, gamepad1.left_stick_x * .5, gamepad1.right_stick_x * .5);

        if(gamepad1.dpad_up) {
            subsystems.drivetrain.startOdometryLift();
        }
        if(gamepad1.dpad_down) {
            subsystems.drivetrain.stopOdometryLift();
        }

        telemetry.update();

    }

}
