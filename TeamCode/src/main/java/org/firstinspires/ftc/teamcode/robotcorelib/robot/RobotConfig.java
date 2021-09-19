package org.firstinspires.ftc.teamcode.robotcorelib.robot;

import org.firstinspires.ftc.teamcode.robotcorelib.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Carousel;

import java.util.ArrayList;


//Configuration class for all subsystems to interface with the system properly.
public class RobotConfig {

    /*
    HardwareMap Reference:
    MOTORS:
        Control Hub Ports:
            0 = front_left
            1 = front_right
            2 = back_left
            3 = back_right
        Expansion Hub Ports:
            0 = odo_left
            1 = odo_strafe
            2 = odo_right
            3 = lift
    SERVOS:
        Control Hub:

        Expansion Hub:
            0 = release
            4 = intake_wall
            5 = duck_servo

    I2C SENSORS:
        Control Hub:

        Expansion Hub:

    ANALOG PORTS:
        Control Hub:

        Expansion Hub:

     */

    public ArrayList<Subsystem> subsystems = new ArrayList<>();

    //localization
    public StandardTrackingWheelLocalizer localizer;

    //Subsystem objects go here
    public Drivetrain drivetrain;
    public Intake intake;
    public Lift lift;
    public Carousel carousel;

    //initialization of subsystems goes here
    public void init() {
        localizer = new StandardTrackingWheelLocalizer(Robot.getHardwareMap());
        drivetrain = new Drivetrain();
        intake = new Intake();
        lift = new Lift();
        carousel = new Carousel();

    }

}
