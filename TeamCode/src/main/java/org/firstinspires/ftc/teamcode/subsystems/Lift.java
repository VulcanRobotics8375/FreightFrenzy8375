package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.math.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Point;
import org.apache.commons.math3.analysis.function.Sigmoid;
import org.firstinspires.ftc.teamcode.robotcorelib.math.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Lift extends Subsystem {
    private DcMotor lift;
    private DcMotor turret;
    private Servo release;
    private Servo linkage;

    SimplePID pid = new SimplePID(0.0005, 0, 0, 1, -1);
    SimplePID turnPID = new SimplePID(0.05,0,0,1,-1);
    private SimplePID liftPID = new SimplePID(0.0005, 0, 0, 1, -1);

    private boolean liftHolding = false;
    private double liftHoldPos;
    private final int LIFT_MIN_POS = 0;
    private final int LIFT_MAX_POS = 800;
    private final double LIFT_CONVERGENCE_SPEED = 0.01;
    private final double LIFT_LIMIT_RANGE = 100;

    private ElapsedTime linkageTimer = new ElapsedTime();
    private final double LINKAGE_MIN_POS = 0.25;
    private final double LINKAGE_MAX_POS = 0.75;
    private final double LINKAGE_STICK_COEF = 0.0007;

    private boolean releaseOpen = false;
    private boolean releaseButton = false;
    private double releasePosOpen = 0.5;
    private double releasePosClosed = 0.05;
    //Temporary until analog input
    private AnalogInput turretAngleAnalog;

    private boolean autoAim = false;

    public final double GOAL_X = 0;
    public final double GOAL_Y = 0;
    public final int BASE_TURRET_POS = 0;
    public final int BASE_LIFT_POS = 0;
    public final double BASE_LINKAGE_POS = 0.01;

    public void init(){
        release = hardwareMap.servo.get("release");
        lift = hardwareMap.dcMotor.get("lift");
        linkage = hardwareMap.servo.get("linkage");
        turret = hardwareMap.dcMotor.get("turret");;
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void run(double liftStick, double turretStick, double linkageStick, boolean turretButton, boolean resetButton, boolean releaseButton) {

        //Auto-aim for auto
        Point target = new Point(GOAL_X, GOAL_Y);
        double turretAngle = turretAngleAnalog.getVoltage();
        if (autoAim) {
            double angleToTarget = Math.atan2(target.x - Robot.getRobotPose().getX(), target.y - Robot.getRobotPose().getY());
            double currentAngle = Robot.getRobotPose().getHeading() + turretAngle;
            turret.setPower(turnPID.run(angleToTarget, currentAngle));
        }

        //Boooooooooooootons.
        if (resetButton) {
            linkage.setPosition(BASE_LINKAGE_POS);
            liftToPosition(BASE_LIFT_POS);
            turretToPosition(BASE_TURRET_POS);

        }

        if (Math.abs(turretStick) > 0.05) {
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret.setPower(turretStick);
        }

        //Lift Code
        int liftPos = lift.getCurrentPosition();

        double liftPower;
        if (liftStick != 0) {
            if (liftHolding) {
                liftPID.reset();
                liftHolding = false;
            }
            if (liftStick > 0) {
                liftPower = liftStick * sigmoid(LIFT_CONVERGENCE_SPEED * (liftPos - (LIFT_MAX_POS - LIFT_LIMIT_RANGE)));
            } else {
                liftPower = liftStick * sigmoid(LIFT_CONVERGENCE_SPEED * (LIFT_LIMIT_RANGE / 2 - liftPos));
            }
        } else {
            if (!liftHolding) {
                liftHoldPos = Range.clip(liftPos, LIFT_MIN_POS, LIFT_MAX_POS);
                liftHolding = true;
            }
            liftPower = liftPID.run(liftHoldPos, liftPos);
        }
        lift.setPower(liftPower);


        //Linkage Code
        double elapsed = linkageTimer.milliseconds();
        double targetPos = linkage.getPosition() + LINKAGE_STICK_COEF * elapsed * linkageStick;
        linkage.setPosition(Range.clip(targetPos, LINKAGE_MIN_POS, LINKAGE_MAX_POS));
        linkageTimer.reset();


        //Hopper Code
        if (releaseButton && !this.releaseButton) {
            this.releaseButton = true;
            releaseOpen = !releaseOpen;
        } else if (!releaseButton && this.releaseButton) {
            this.releaseButton = false;
        }

        if (releaseOpen) {
            release.setPosition(releasePosOpen);
        } else {
            release.setPosition(releasePosClosed);
        }
    }


    public void turretToPosition(int pos){
        turret.setTargetPosition(pos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }

    public void liftToPosition(int pos) {
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }

    public void test(double liftStick) {
        lift.setPower(liftStick);
        telemetry.addData("lift pos", lift.getCurrentPosition());
    }

    public static double sigmoid(double x) {
        return 1 / (1 + Math.exp(x));
    }
}