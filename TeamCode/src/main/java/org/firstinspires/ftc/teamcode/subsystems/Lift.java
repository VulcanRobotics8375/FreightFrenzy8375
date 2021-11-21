package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.math.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Lift extends Subsystem {
    private DcMotor lift;
    private Servo release;
    private Servo linkage;

    private boolean hold = false;
    private int holdPosition;
    PID pid = new PID(0.0005, 0, 0, 1, -1);

    private double buttonOn = -1;
    private boolean buttonPress = false;

    private boolean linkageButton = false;
    private double linkageOn = -1;

    private final int LIMIT_RANGE = 300;
    private final int MAX_HEIGHT = 1100;
    private final double CONVERGENCE_SPEED = 8.0 / (double) LIMIT_RANGE;
    private final double CLOSED_POS = 0.01;
    private final double OPENED_POS = 0.65;


    public void init(){
        release = hardwareMap.servo.get("release");
        lift = hardwareMap.dcMotor.get("lift");
        linkage = hardwareMap.servo.get("linkage");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void run(double stickPower, boolean buttonPress, boolean linkageButton) {
        int pos = lift.getCurrentPosition();
        double outputPower;
        if(stickPower < 0) {
            stickPower *= 0.5;
        }

        // Sigmoid
        //value to tune here is the numerator-- higher number == faster acceleration curve
        if(stickPower > 0) {
            hold = false;
            outputPower = stickPower / (1 + Math.exp(CONVERGENCE_SPEED * (pos - (MAX_HEIGHT - (LIMIT_RANGE / 2.0)))));
        } else if(stickPower < 0) {
            hold = false;
            outputPower = stickPower / (1 + Math.exp(CONVERGENCE_SPEED * (LIMIT_RANGE/2.0 - pos)));
        } else {
            if(!hold) {
                holdPosition = pos;
                hold = true;
            }
            outputPower = pid.run(holdPosition, pos);
        }

        lift.setPower(outputPower);

        if(buttonPress && !this.buttonPress) {
            this.buttonPress = true;
            buttonOn *= -1;
        }
        if(!buttonPress && this.buttonPress) {
            this.buttonPress = false;
        }
        if(buttonOn > 0) {
            release.setPosition(OPENED_POS);
        }
        if(buttonOn < 0) {
            release.setPosition(CLOSED_POS);
        }

//        if(pos < 150 && release.getPosition() != CLOSED_POS){
//            release.setPosition(CLOSED_POS);
//        }

        if(linkageButton && !this.linkageButton){
            linkageOn *= -1;
            this.linkageButton = true;
        }
        if(!linkageButton && this.linkageButton){
            this.linkageButton = false;
        }
        if(linkageOn > 0){
            linkage.setPosition(1.0);
        }
        if(linkageOn < 0){
            linkage.setPosition(0.45);
        }

        telemetry.addData("lift pos", pos);
//        telemetry.addData("hold", hold);

    }


    public void test(double stickPower) {
        lift.setPower(stickPower);
        telemetry.addData("lift pos", lift.getCurrentPosition());
    }


}
