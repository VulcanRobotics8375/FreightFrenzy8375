package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Intake extends Subsystem {
    private DcMotorEx intakeMotor;
    private Servo extendServo1;
    private Servo extendServo2;
    private Servo rotateServo;

    public boolean lastFlip = false;
    public double flipOn = -1;
    public final double depositPosition = 0.15;
    public final double intakePosition = 0.9;
    public final double resetPosition = 0.99;

    private boolean intakeButton;
    private boolean indexed = false;

    public boolean lastExtend = false;
    public double extendOn = -1;
    private final double extendBackPosition = 0.28;
    private final double extendDepositPosition = 0.32;
    private double extendForwardPosition = 0.05;

    public final double INTAKE_POWER = 1;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        extendServo1 = hardwareMap.servo.get("extend_one");
        extendServo2 = hardwareMap.servo.get("extend_two");
        rotateServo = hardwareMap.servo.get("flip");

        extendServo1.setDirection(Servo.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    ElapsedTime timer = new ElapsedTime();

    private IntakeState intakeState = IntakeState.RESET;
    public void run(boolean intake, boolean reset, boolean liftReady) {
        if(intake) {
            intakeState = IntakeState.INTAKING;
        }
        if(reset) {
            intakeState = IntakeState.RESET;
        }

        switch (intakeState) {
            case INTAKING:
                rotateServo.setPosition(intakePosition);
                extendServo1.setPosition(extendForwardPosition);
                extendServo2.setPosition(extendForwardPosition);
                intakeMotor.setPower(INTAKE_POWER);
                if(intakeMotor.getCurrent(CurrentUnit.AMPS) > 6.0) {
                    if(timer.milliseconds() > 100) {
                        timer.reset();
                        gamepad1.rumble(500);
                        intakeState = IntakeState.INDEXED;
                    }
                } else {
                    timer.reset();
                }
                break;
            case INDEXED:
                rotateServo.setPosition(depositPosition);
                extendServo1.setPosition(extendDepositPosition);
                extendServo2.setPosition(extendDepositPosition);
                intakeMotor.setPower(INTAKE_POWER * 0.8);
                if(liftReady && timer.seconds() > 0.7) {
                    gamepad2.rumble(500);
                    intakeState = IntakeState.DEPOSIT;
                }
                break;
            case DEPOSIT:
                rotateServo.setPosition(depositPosition);
                extendServo1.setPosition(extendDepositPosition);
                extendServo2.setPosition(extendDepositPosition);
                intakeMotor.setPower(INTAKE_POWER * -1.0);
                break;
            case RESET:
                rotateServo.setPosition(resetPosition);
                extendServo1.setPosition(extendBackPosition);
                extendServo2.setPosition(extendBackPosition);
                intakeMotor.setPower(0.0);
                break;
        }

        telemetry.addData("intake current", intakeMotor.getCurrent(CurrentUnit.AMPS));

    }

    public enum IntakeState {
        INTAKING,
        INDEXED,
        DEPOSIT,
        RESET
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public void test(boolean intake, boolean outake, boolean extend, boolean flip) {
        if (intake) {
            intakeMotor.setPower(INTAKE_POWER);
        }
        else if (outake) {
            intakeMotor.setPower(-1 * INTAKE_POWER);
        } else {
            intakeMotor.setPower(0.0);
        }

       // extension servos
        if(extend && !lastExtend) {
            extendOn *= -1;
            lastExtend = true;
        }
        else if (!extend && lastExtend) {
            lastExtend = false;
        }
        if (extendOn > 0) {
            extendServo1.setPosition(extendForwardPosition);
            extendServo2.setPosition(extendForwardPosition);
        }
        else if (extendOn < 0) {
            extendServo1.setPosition(extendBackPosition);
            extendServo2.setPosition(extendBackPosition);
        }
//        if(extend) {
//            extendServo1.setPosition(extendForwardPosition);
//            extendServo2.setPosition(extendForwardPosition);
//        } else {
//
//            extendServo1.setPosition(extendBackPosition);
//            extendServo2.setPosition(extendBackPosition);
//        }

        //flip servo
        if(flip && !lastFlip ) {
            flipOn *= -1;
            lastFlip = true;
        }
        else if (!flip && lastFlip) {
            lastFlip = false;
        }
        if (flipOn > 0) {
            rotateServo.setPosition(intakePosition);
        }
        else if (flipOn < 0) {
            rotateServo.setPosition(depositPosition);
        }
        
    }

    public double getExtendForwardPosition() {
        return extendForwardPosition;
    }
    public void setExtendForwardPosition(double pos) {
        this.extendForwardPosition = pos;
    }

}
