package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.math.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.AnalogEncoder;

public class Lift extends Subsystem {
    private DcMotorEx lift;
    private DcMotorEx turret;
    private Servo release;
    private Servo linkageOne, linkageTwo;

    private SimplePID liftPID = new SimplePID(0.0005, 0, 0, -1, 1);
    private boolean liftHolding = false;
    private double liftTargetPos;
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

    SimplePID turretPID = new SimplePID(0.05,0,0,1,-1);
    private AnalogEncoder turretAngleAnalog; //Temporary until analog input
    private boolean turretHolding = false;
    private double turretTargetPos;

    private boolean autoAim = false;

    public final double GOAL_X = 0;
    public final double GOAL_Y = 0;
    public final int BASE_TURRET_POS = 0;
    public final int BASE_LIFT_POS = 0;
    public final double BASE_LINKAGE_POS = 0.01;

    public void init() {
        release = hardwareMap.servo.get("release");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        linkageOne = hardwareMap.servo.get("linkage_one");
        linkageTwo = hardwareMap.servo.get("linkage_two");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret does not have a quadrature encoder
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        linkageTwo.setDirection(Servo.Direction.REVERSE);
        turretAngleAnalog = hardwareMap.get(AnalogEncoder.class, "turret_encoder");
    }

    public void run(double liftStick, double turretStick, double linkageStick, boolean turretButton, boolean resetButton, boolean releaseButton) {
        //Lift
        int liftPos = lift.getCurrentPosition();

        double liftPower;
        if (liftStick != 0) {
            if (liftHolding) {
                liftPID.reset();
                liftHolding = false;
            }
            //Sigmoid Motor Limits
            if (liftStick > 0) {
                liftPower = liftStick * sigmoid(LIFT_CONVERGENCE_SPEED * (liftPos - (LIFT_MAX_POS - LIFT_LIMIT_RANGE)));
            } else {
                liftPower = liftStick * sigmoid(LIFT_CONVERGENCE_SPEED * (LIFT_LIMIT_RANGE / 2 - liftPos));
            }
        } else {
            if (!liftHolding) {
                liftTargetPos = Range.clip(liftPos, LIFT_MIN_POS, LIFT_MAX_POS);
                liftHolding = true;
            }
            liftPower = liftPID.run(liftTargetPos, liftPos);
        }
        lift.setPower(liftPower);

        //Turret
        //have to call AnalogEncoder.update() every loop
        turretAngleAnalog.update();
        //returns the number of rotations reported by the encoder
        double turretRotations = turretAngleAnalog.getCurrentPosition(AnalogEncoder.Mode.INCREMENTAL);
        double turretAngle = turretAngleAnalog.getVoltage();

        double turretPower;
        if (turretStick != 0) {
            if (turretHolding) {
                turretPID.reset();
                turretHolding = false;
            }
            turretPower = turretStick;
        } else {
            if (!turretHolding) {
                turretTargetPos = turretRotations;
                turretHolding = true;
            }
            turretPower = turretPID.run(turretTargetPos, turretRotations);
        }
//        should we put autoAim in the autopath class?
//        if (autoAim) { //Auto-aim for auto
//            Point target = new Point(GOAL_X, GOAL_Y);
//            double angleToTarget = Math.atan2(target.x - Robot.getRobotPose().getX(), target.y - Robot.getRobotPose().getY());
//            double currentAngle = Robot.getRobotPose().getHeading() + turretAngle;
//            turretPower = turretPID.run(angleToTarget, currentAngle);
//        }
        turret.setPower(turretPower);


        //Linkage
        double elapsed = linkageTimer.milliseconds();
        double targetPos = linkageOne.getPosition() + LINKAGE_STICK_COEF * elapsed * linkageStick;
        linkageOne.setPosition(Range.clip(targetPos, LINKAGE_MIN_POS, LINKAGE_MAX_POS));
        linkageTimer.reset();


        //Hopper
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


        //Run To Position
        if (resetButton) {
            linkageOne.setPosition(BASE_LINKAGE_POS);
            liftToPosition(BASE_LIFT_POS);
            turretToPosition(BASE_TURRET_POS);
        }
    }


    //@matt this isnt going to work, our turret isnt using a quadrature encoder so we cant use any encoder modes (RUN_TO_POSITION, RUN_USING_ENCODER)
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

    public void test(double liftStick, double turretStick, boolean linkageButton, boolean releaseButton) {
        int liftPos = lift.getCurrentPosition();

        double liftPower;
        if (liftStick != 0) {
            if (liftHolding) {
                liftPID.reset();
                liftHolding = false;
            }
            liftPower = liftStick;
        } else {
            if (!liftHolding) {
                liftTargetPos = Range.clip(liftPos, LIFT_MIN_POS, LIFT_MAX_POS);
                liftHolding = true;
            }
            liftPower = liftPID.run(liftTargetPos, liftPos);
        }
        lift.setPower(liftPower);

        turret.setPower(turretStick);

        if(linkageButton) {
            linkageOne.setPosition(0.8);
            linkageTwo.setPosition(0.8);
        }
        else {
            linkageTwo.setPosition(0.1);
            linkageOne.setPosition(0.1);
        }
        if(releaseButton) {
            release.setPosition(0.23);
        } else {
            release.setPosition(0.6);
        }
        turretAngleAnalog.update();

        telemetry.addData("turret pos incremental", turretAngleAnalog.getCurrentPosition(AnalogEncoder.Mode.INCREMENTAL));
        telemetry.addData("turret pos", turretAngleAnalog.getVoltage());

        telemetry.addData("turret encoder max voltage", turretAngleAnalog.getMaxVoltage());

        telemetry.addData("lift pos", lift.getCurrentPosition());
    }

    public static double sigmoid(double x) {
        return 1 / (1 + Math.exp(-x));
    }
}