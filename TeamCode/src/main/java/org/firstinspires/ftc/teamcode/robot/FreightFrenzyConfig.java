package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class FreightFrenzyConfig extends RobotConfig {

    public Drivetrain drivetrain;
    public Intake intake;
    public Lift lift;

    @Override
    public void init() {
        subsystems.clear();
        drivetrain = new Drivetrain();
        intake = new Intake();
        lift = new Lift();
    }
}
