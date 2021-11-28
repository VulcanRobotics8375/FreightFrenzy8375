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

    private double releaseOn = -1;
    private boolean releaseButton = false;

    private final int BOTTOM_LEVEL = 0;
    private final int FIRST_LEVEL = 0;
    private final int SECOND_LEVEL = 300;
    private final int THIRD_LEVEL = 600;
    private final int LIMIT_RANGE = 200;
    private final int MAX_HEIGHT = 1100;
    private final double CONVERGENCE_SPEED = 8.0 / (double) LIMIT_RANGE;
    private final double LINKAGE_STICK_COEF = 0.005;
    private final double LINKAGE_OPENED = 1.0;
    private final double LINKAGE_CLOSED = 0.49;
    private final double RELEASE_CLOSED = 0.01;
    private final double RELEASE_OPENED = 0.45;

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

    public void run(double liftStick, boolean releaseButton, double linkageStick, boolean firstLevel, boolean secondLevel, boolean thirdLevel, boolean reset) {
        double linkagePos = this.linkagePos;
        if(runningToPosition) {
        } else if(reset) {
            liftToPosition(BOTTOM_LEVEL);
            linkagePos = LINKAGE_CLOSED;
        } else if(firstLevel) {
            liftToPosition(FIRST_LEVEL);
            linkagePos = LINKAGE_OPENED;
        } else if(secondLevel) {
            liftToPosition(SECOND_LEVEL);
            linkagePos = LINKAGE_OPENED;
        } else if(thirdLevel) {
            liftToPosition(THIRD_LEVEL);
            linkagePos = LINKAGE_OPENED;
        }

        if(runningToPosition && (reset || firstLevel || secondLevel || thirdLevel)) {
            if(reset && lift.getTargetPosition() != BOTTOM_LEVEL && linkage.getPosition() != LINKAGE_CLOSED) {
                lift.setTargetPosition(BOTTOM_LEVEL);
                linkagePos = LINKAGE_CLOSED;
            }
            if(firstLevel && lift.getTargetPosition() != FIRST_LEVEL) {
                lift.setTargetPosition(FIRST_LEVEL);
                linkagePos = LINKAGE_OPENED;
            } else if(secondLevel && lift.getTargetPosition() != SECOND_LEVEL) {
                lift.setTargetPosition(SECOND_LEVEL);
                linkagePos = LINKAGE_OPENED;
            } else if(thirdLevel && lift.getTargetPosition() != THIRD_LEVEL) {
                lift.setTargetPosition(THIRD_LEVEL);
                linkagePos = LINKAGE_OPENED;
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
            releaseOn *= -1;
        }
        if(!releaseButton && this.releaseButton) {
            this.releaseButton = false;
        }
        if(releaseOn > 0) {
            release.setPosition(RELEASE_OPENED);
        }
        if(releaseOn < 0) {
            release.setPosition(RELEASE_CLOSED);
        }

//        if(pos < 150 && release.getPosition() != RELEASE_CLOSED){
//            release.setPosition(RELEASE_CLOSED);
//        }
        

        double elapsed = linkageTimer.milliseconds();

        boolean nearOpened = !(linkagePos < LINKAGE_OPENED - LINKAGE_STICK_COEF * linkageStick);
        boolean nearClosed = !(linkagePos > LINKAGE_CLOSED + LINKAGE_STICK_COEF * linkageStick);
        if(linkageStick > 0 && !nearOpened) {
            linkagePos = this.linkagePos + LINKAGE_STICK_COEF * linkageStick * elapsed;
        } else if(linkageStick < 0 && !nearClosed) {
            linkagePos = this.linkagePos + LINKAGE_STICK_COEF * linkageStick * elapsed;
        }
        linkageTimer.reset();

        linkage.setPosition(linkagePos);

        telemetry.addData("linkage set position", this.linkagePos + LINKAGE_STICK_COEF * linkageStick * elapsed);

        this.linkagePos = linkagePos;

        telemetry.addData("linkage stick", linkageStick);
//        telemetry.addData("lift pos", pos);
//        telemetry.addData("hold", hold);

    }


    public void test(double liftStick) {
        lift.setPower(liftStick);
        telemetry.addData("lift pos", lift.getCurrentPosition());
    }

    public void liftToPosition(int position) {
        lift.setTargetPosition(position);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        runningToPosition = true;
    }


}
