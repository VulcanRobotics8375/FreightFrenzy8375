package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.HardwarePrecision;

public class Intake extends Subsystem {
    private DcMotorEx intakeMotor;
    private Servo extendServo1;
    private Servo extendServo2;
    private Servo rotateServo;

    public boolean lastFlip = false;
    public double flipOn = -1;
    public final double depositPosition = 0.2;
    public final double intakePosition = 0.8;


    public boolean lastExtend = false;
    public double extendOn = -1;
    public final double extendBackPosition = 0.2;
    public final double extendForwardPosition = 0.8;

    public final double INTAKE_POWER = 1;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        extendServo1 = hardwareMap.servo.get("extendServo1");
        extendServo2 = hardwareMap.servo.get("extendServo2");
        rotateServo = hardwareMap.servo.get("rotateServo");

        extendServo1.setPosition(extendBackPosition);
        extendServo2.setPosition(extendBackPosition);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void run(boolean intake, boolean outake, boolean extend, boolean flip ) {
        if (intake) {
            intakeMotor.setPower(INTAKE_POWER);
        }
        else if (outake) {
            intakeMotor.setPower(-1 * INTAKE_POWER);
        }

       // extension servos
        if(extend && !lastExtend ) {
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




}
