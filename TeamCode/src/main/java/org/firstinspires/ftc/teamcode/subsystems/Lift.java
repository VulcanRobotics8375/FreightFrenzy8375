package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotcorelib.math.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Lift extends Subsystem {


    private final double CONVERGENCE_SPEED = 8.0 / (double) LIMIT_RANGE;
    private final double LINKAGE_STICK_COEF = 0.0007;
    private final double LINKAGE_OPENED = 1.0;
    private final double LINKAGE_CLOSED = 0.49;
    private final double RELEASE_CLOSED = 0.01;
    private final double RELEASE_OPENED = 0.45;
    private final double DOWN_SPEED = 0.65;

    private int liftPosOffset = 0;

    private double linkagePos = LINKAGE_CLOSED;
    private double releasePos = RELEASE_CLOSED;

    private int liftPos = 0;

    private ElapsedTime linkageTimer = new ElapsedTime();
    private ElapsedTime linkageResetTimer = new ElapsedTime();

    public void init(){
        release = hardwareMap.servo.get("release");
        lift = hardwareMap.dcMotor.get("lift");
        linkage = hardwareMap.servo.get("linkage");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        runningToPosition = false;
    }

}
