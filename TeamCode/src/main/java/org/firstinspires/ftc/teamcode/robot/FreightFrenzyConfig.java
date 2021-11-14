package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Cap;

public class FreightFrenzyConfig extends RobotConfig {

    public Drivetrain drivetrain;
    public Intake intake;
    public Lift lift;
    public Carousel carousel;
    public Cap cap;

    @Override
    public void init() {
        subsystems.clear();
        drivetrain = new Drivetrain();
        intake = new Intake();
        lift = new Lift();
        carousel = new Carousel();
        cap = new Cap();
    }
}
