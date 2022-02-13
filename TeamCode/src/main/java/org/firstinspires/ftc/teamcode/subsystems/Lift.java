package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotcorelib.math.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Lift extends Subsystem {
    private DcMotor lift;
    private DcMotor turret;
    private Servo release;
    private Servo linkage;

    PID pid = new PID(0.0005, 0, 0, 1, -1);

    private double releaseOpen = -1;
    private boolean releaseButton = false;
    private double releasePosOpen = 0.5;
    private double releasePosClosed = 0.05;

    public void init(){
        release = hardwareMap.servo.get("release");
        lift = hardwareMap.dcMotor.get("lift");
        linkage = hardwareMap.servo.get("linkage");
        turret = hardwareMap.dcMotor.get("turret");;
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void run(double liftStick, double turretStick, boolean releaseButton) {

        //Hopper Code
        if(releaseButton && !this.releaseButton) {
            this.releaseButton = true;
            releaseOpen *= -1;
        }
        if(!releaseButton && this.releaseButton) {
            this.releaseButton = false;
        }
        if(releaseOpen > 0){
            release.setPosition(releasePosOpen);
        }
        if(releaseOpen < 0){
            release.setPosition(releasePosClosed);
        }

    }

    public void test(double liftStick) {
        lift.setPower(liftStick);
        telemetry.addData("lift pos", lift.getCurrentPosition());
    }
}