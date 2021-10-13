package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
public class Lift extends Subsystem {
    private DcMotor lift;
    private Servo release;
    private Servo cap;

    private boolean hold = false;
    private int holdPosition;
    private boolean open = false;
    private boolean buttonRelease = false;
    private boolean goingToPosition = false;
    private int liftSetHeight = 0;
    private boolean capButtonOn = false;
    private boolean capButtonToggle = false;

    private final double HOLD_POS_GAIN = 0.0;
    private final int LIMIT_RANGE = 300;
    private final int MAX_HEIGHT = 600;
    private final double CONVERGENCE_SPEED = 8.0 / (double) LIMIT_RANGE;
    
    private final double CLOSED_POS = 0.25;
    private final double OPENED_POS = 0.9;
    
    private final int FIRST_LEVEL = 200;
    private final int SECOND_LEVEL = 350;
    private final int THIRD_LEVEL = 500;


    public void init(){
        release = hardwareMap.servo.get("release");
        lift = hardwareMap.dcMotor.get("lift");
        cap = hardwareMap.servo.get("cap");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run(double stickPower, boolean buttonRelease, boolean buttonA, boolean buttonB, boolean buttonY, boolean capButton) {
        int pos = lift.getCurrentPosition();
        double outputPower;
        if(stickPower < 0) {
            stickPower *= 0.3;
        }

        // Sigmoid
        //value to tune here is the numerator-- higher number == faster acceleration curve
        if(stickPower > 0) {
            if(goingToPosition) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                goingToPosition = false;
            }
            hold = false;
            outputPower = stickPower / (1 + Math.exp(CONVERGENCE_SPEED * (pos - (MAX_HEIGHT - (LIMIT_RANGE / 2.0)))));
        } else if(stickPower < 0) {
            if(goingToPosition) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                goingToPosition = false;
            }
            hold = false;
            outputPower = stickPower / (1 + Math.exp(CONVERGENCE_SPEED * (LIMIT_RANGE/2.0 - pos)));
        } else {
            if(!hold) {
                holdPosition = pos;
                hold = true;
            }
            if(goingToPosition) {
                outputPower = 1;
            } else {
                int error = holdPosition - pos;
                outputPower = error * HOLD_POS_GAIN;
            }
            if((buttonA || buttonB || buttonY) && !goingToPosition) {
                goingToPosition = true;
                if(buttonA) {
                    liftSetHeight = 0;
                    lift.setTargetPosition(FIRST_LEVEL);
                } else if(buttonB) {
                    liftSetHeight = 1;
                    lift.setTargetPosition(SECOND_LEVEL);
                }
                else {
                    liftSetHeight = 2;
                    lift.setTargetPosition(THIRD_LEVEL);
                }
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outputPower = 1;

            }




        }

        lift.setPower(outputPower);
        if(buttonRelease && !this.buttonRelease) {
            this.buttonRelease = true;
            open = !open;
        }
        if(!buttonRelease && this.buttonRelease) {
            this.buttonRelease = false;
        }
        if(open) {
            release.setPosition(OPENED_POS);
        } else {
            release.setPosition(CLOSED_POS);
        }

        if(capButton && !capButtonToggle) {
            capButtonOn = !capButtonOn;
            capButtonToggle = true;
        }
        if(!capButton && capButtonToggle) {
            capButtonToggle = false;
        }

        if(capButtonOn) {
            cap.setPosition(0.59);
        } else {
            cap.setPosition(0.2);
        }

//        telemetry.addData("lift pos", pos);
//        telemetry.addData("hold", hold);

    }
    

    public void test(double stickPower) {
        lift.setPower(stickPower);
        telemetry.addData("lift pos", lift.getCurrentPosition());
    }


}
