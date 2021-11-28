package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotcorelib.math.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Lift extends Subsystem {
    private DcMotor lift;
    private Servo release;
    private Servo linkage;

    private boolean runningToPosition = false;
    private boolean hold = false;
    private int holdPosition;
    PID pid = new PID(0.0005, 0, 0, 1, -1);

    private double buttonOn = -1;
    private boolean releaseButton = false;

    private final int FIRST_LEVEL = 300;
    private final int SECOND_LEVEL = 600;
    private final int THIRD_LEVEL = 900;
    private final int LIMIT_RANGE = 200;
    private final int MAX_HEIGHT = 1100;
    private final double CONVERGENCE_SPEED = 8.0 / (double) LIMIT_RANGE;
    private final double LINKAGE_STICK_COEF = 0.0005;
    private final double LINKAGE_OPENED = 1.0;
    private final double LINKAGE_CLOSED = 0.49;
    private final double RELEASE_CLOSED = 0.01;
    private final double RELEASE_OPENED = 0.45;

    private boolean linkageButton = false;
    private double linkageOn = -1;
    private double linkagePos = LINKAGE_CLOSED;

    private ElapsedTime linkageTimer = new ElapsedTime();

    public void init(){
        release = hardwareMap.servo.get("release");
        lift = hardwareMap.dcMotor.get("lift");
        linkage = hardwareMap.servo.get("linkage");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void run(double liftStick, boolean releaseButton, boolean linkageButton, double linkageStick, boolean firstLevel, boolean secondLevel, boolean thirdLevel) {
        if(runningToPosition) { }
        else if(firstLevel) { runToPosition(FIRST_LEVEL); }
        else if(secondLevel) { runToPosition(SECOND_LEVEL); }
        else if(thirdLevel) { runToPosition(THIRD_LEVEL); }

        if(runningToPosition && (firstLevel || secondLevel || thirdLevel)) {
            if(firstLevel && lift.getTargetPosition() != FIRST_LEVEL) {
                lift.setTargetPosition(FIRST_LEVEL);
            } else if(secondLevel && lift.getTargetPosition() != SECOND_LEVEL) {
                lift.setTargetPosition(SECOND_LEVEL);
            } else if(thirdLevel && lift.getTargetPosition() != THIRD_LEVEL) {
                lift.setTargetPosition(THIRD_LEVEL);
            }
        }

        if(Math.abs(liftStick) > 0.05 && runningToPosition) {
            runningToPosition = false;
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        int pos = lift.getCurrentPosition();

        if(Math.abs(liftStick) > 0.05 || !runningToPosition) {
            double outputPower;
            // Sigmoid
            //value to tune here is the numerator-- higher number == faster acceleration curve
            if (liftStick > 0) {
                hold = false;
                outputPower = liftStick / (1 + Math.exp(CONVERGENCE_SPEED * (pos - (MAX_HEIGHT - (LIMIT_RANGE / 2.0)))));
            } else if (liftStick < 0) {
                hold = false;
                outputPower = liftStick / (1 + Math.exp(CONVERGENCE_SPEED * (LIMIT_RANGE / 2.0 - pos)));
            } else {
                if (!hold) {
                    holdPosition = pos;
                    hold = true;
                }
                outputPower = pid.run(holdPosition, pos);
            }

            lift.setPower(outputPower);
        }

        if(releaseButton && !this.releaseButton) {
            this.releaseButton = true;
            buttonOn *= -1;
        }
        if(!releaseButton && this.releaseButton) {
            this.releaseButton = false;
        }
        if(buttonOn > 0) {
            release.setPosition(RELEASE_OPENED);
        }
        if(buttonOn < 0) {
            release.setPosition(RELEASE_CLOSED);
        }

//        if(pos < 150 && release.getPosition() != RELEASE_CLOSED){
//            release.setPosition(RELEASE_CLOSED);
//        }

        if(linkageButton && !this.linkageButton){
            linkageOn *= -1;
            this.linkageButton = true;
        }
        if(!linkageButton && this.linkageButton){
            this.linkageButton = false;
        }

        double linkagePos = this.linkagePos;
        if(linkageOn > 0 && this.linkageButton){
            linkagePos = LINKAGE_OPENED;
        }
        if(linkageOn < 0 && this.linkageButton){
            linkagePos = LINKAGE_CLOSED;
        }

        double elapsed = linkageTimer.milliseconds();

        if(linkageStick != 0 && !this.linkageButton) {
            linkagePos = this.linkagePos + LINKAGE_STICK_COEF * linkageStick * elapsed;
        }
        linkageTimer.reset();

        linkage.setPosition(linkagePos);
        this.linkagePos = linkagePos;

        telemetry.addData("linkage stick", linkageStick);
//        telemetry.addData("lift pos", pos);
//        telemetry.addData("hold", hold);

    }


    public void test(double liftStick) {
        lift.setPower(liftStick);
        telemetry.addData("lift pos", lift.getCurrentPosition());
    }

    public void setLinkagePosition(double pos) {
        linkage.setPosition(pos);
    }
    public void setReleasePosition(double pos) {
        release.setPosition(pos);
    }

    public void runToPosition(int position) {
        lift.setTargetPosition(position);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        runningToPosition = true;
    }


}
