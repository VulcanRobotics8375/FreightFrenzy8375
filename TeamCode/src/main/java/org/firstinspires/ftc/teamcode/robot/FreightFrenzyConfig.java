package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class FreightFrenzyConfig extends RobotConfig {

    public Drivetrain drivetrain;
    public Intake intake;

    @Override
    public void init() {
        subsystems.clear();
        drivetrain = new Drivetrain();
        intake = new Intake();
    }
}
