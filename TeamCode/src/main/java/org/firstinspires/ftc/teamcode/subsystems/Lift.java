package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
public class Lift extends Subsystem {
    private DcMotor lift;
    private Servo release;

    private boolean hold = false;
    private int holdPosition;
    private boolean open = false;
    private boolean buttonRelease = false;
    private boolean goingToPosition = false;

    private final double HOLD_POS_GAIN = 0.0005;
    private final int LIMIT_RANGE = 300;
    private final int MAX_HEIGHT = 600;
    private final double CONVERGENCE_SPEED = 8.0 / (double) LIMIT_RANGE;
    
    private final double CLOSED_POS = 0.05;
    private final double OPENED_POS = 0.65;
    
    private final int FIRST_LEVEL = 200;
    private final int SECOND_LEVEL = 350;
    private final int THIRD_LEVEL = 500;


    public void init(){
        release = hardwareMap.servo.get("release");
        lift = hardwareMap.dcMotor.get("lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void run(double stickPower, boolean buttonRelease, boolean buttonA, boolean buttonB, boolean buttonY) {
        int pos = lift.getCurrentPosition();
        double outputPower;
        if(stickPower < 0) {
            stickPower *= 0.2;
        }
        if(hold){
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

            int error = holdPosition - pos;
            outputPower = error * HOLD_POS_GAIN;

            if(buttonA) {
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setTargetPosition(FIRST_LEVEL);
                outputPower = 1;
            } else if(buttonB) {
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setTargetPosition(SECOND_LEVEL);
                outputPower = 1;
            } else if(buttonY) {
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setTargetPosition(THIRD_LEVEL);
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
        if(open && pos > 150) {
            release.setPosition(OPENED_POS);
        } else {
            release.setPosition(CLOSED_POS);
        }

        telemetry.addData("lift pos", pos);
//        telemetry.addData("hold", hold);

    }
    

    public void test(double stickPower) {
        lift.setPower(stickPower);
        telemetry.addData("lift pos", lift.getCurrentPosition());
    }


}
