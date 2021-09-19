package org.firstinspires.ftc.teamcode.robotcorelib.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;

public abstract class TeleOpPipeline extends OpMode {
    protected RobotConfig subsystems;

    @Override
    public void init() {
        Robot.init(hardwareMap, telemetry);
        subsystems = Robot.getConfiguration();
    }

    public void start() {

    }

    public abstract void loop();

}
