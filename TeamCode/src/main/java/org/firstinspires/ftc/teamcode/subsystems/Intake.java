package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
public class Intake extends Subsystem {
    private DcMotor intake;
    private Servo intakeWall;

    private boolean intakeToggle = false;
    private boolean intakeWallOn = false;
    private final double intakePower = 1.0;
    private final double intakeWallClosed = 0.05;
    private final double intakeWallOpen = 0.5;

    public void init(){
        intake = hardwareMap.dcMotor.get("odo_right");
        intakeWall = hardwareMap.servo.get("intake_wall");
    }

    public void run(boolean buttonIntake, boolean buttonOuttake, boolean intakeToggle){
        if(buttonIntake){
            intake.setPower(intakePower);
        }else if (buttonOuttake){
            intake.setPower(-intakePower);
        }else {
            intake.setPower(0);
        }

        if(intakeToggle && !this.intakeToggle) {
            this.intakeToggle = true;
            intakeWallOn = !intakeWallOn;
        }
        if(!intakeToggle && this.intakeToggle) {
            this.intakeToggle = false;
        }

        if(intakeWallOn) {
            intakeWall.setPosition(intakeWallClosed);
        }
        else {
            intakeWall.setPosition(intakeWallOpen);
        }


    }

}
