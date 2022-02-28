package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.math.KalmanFilter;
import org.firstinspires.ftc.teamcode.robotcorelib.math.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.AnalogEncoder;

public class Lift extends Subsystem {
    private DcMotorEx lift;
    private DcMotorEx turret;
    private Servo release;
    private Servo linkageOne, linkageTwo;

    SimplePID liftPID = new SimplePID(0.005, 0.0, 0.0, 1.0, -1.0);
    private double liftTargetPos;
    private boolean liftHolding = false;
    private LiftMin LIFT_MIN = new LiftMin();
    private final int LIFT_MAX_POS = 650;
    private final int LIFT_CLEARED_POS = 400;
    private final int LIFT_ALLIANCE_POS = 475;
    private final double LIFT_CONVERGENCE_SPEED = 0.01;
    private final double LIFT_LIMIT_RANGE = 100.0;

    private ElapsedTime linkageTimer = new ElapsedTime();
    private boolean linkageButton = false;
    private boolean linkageOpen = false;
    private final double LINKAGE_MIN_POS = 0.25;
    private final double LINKAGE_MAX_POS = 0.75;
    private final double LINKAGE_STICK_COEF = 0.0007;

    private boolean releaseOpen = false;
    private boolean releaseButton = false;
    private double releasePosOpen = 0.5;
    private double releasePosClosed = 0.05;

    SimplePID turretPID = new SimplePID(0.05,0.0,0.0,1.0,-1.0);
    private AnalogInput turretAngleAnalog; //Temporary until analog input
    private boolean turretHolding = false;
    private double turretTargetPos;
    private int turret90Degrees = -364;

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
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret does not have a quadrature encoder
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
        linkageTwo.setDirection(Servo.Direction.REVERSE);
        turretAngleAnalog = hardwareMap.get(AnalogInput.class, "turret_encoder");
        
        PIDFCoefficients stockLiftCoeff = lift.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
//        lift.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(stockLiftCoeff.p, stockLiftCoeff.i, stockLiftCoeff.d, 0.1));


    }

//    public void run(double liftStick, double turretStick, double linkageStick, boolean turretButton, boolean resetButton, boolean releaseButton) {
//        // Get Positions!
//        int liftPos = lift.getCurrentPosition();
//
//        //have to call AnalogEncoder.update() every loop
//        turretAngleAnalog.update();
//        //returns the number of rotations reported by the encoder
//        double turretRotations = turretAngleAnalog.getCurrentPosition(AnalogEncoder.Mode.INCREMENTAL);
//        double turretAngle = turretAngleAnalog.getVoltage();
//
//        //TODO linkage kinematics
//
//        //TODO Convert Analog Encoder Output to Angle(Radians)
//
//        //Lift
//        double liftPower;
////        double feedForwardCoeff = liftPos > 50 ? 0.15 : 0.0;
//        if (liftStick != 0) {
//            //Sigmoid Motor Limits
//            if (liftStick > 0) {
//                liftPower = liftStick * sigmoid(LIFT_CONVERGENCE_SPEED * (liftPos - (LIFT_MAX_POS - LIFT_LIMIT_RANGE)));
//            } else {
//                liftPower = liftStick * sigmoid(LIFT_CONVERGENCE_SPEED * (LIFT_LIMIT_RANGE / 2 - liftPos));x
//            }
//        } else {
//            if (!liftHolding) {
//                liftTargetPos = Range.clip(liftPos, LIFT_MIN.value(turretAngle), LIFT_MAX_POS);
//                liftHolding = true;
//            }
//            liftPower = liftPID.run(liftTargetPos, liftPos);
//        }
//        lift.setPower(liftPower);
//
//        //Turret
//        //have to call AnalogEncoder.update() every loop
////        turretAngleAnalog.update();
//        //returns the number of rotations reported by the encoder
//        double turretRotations = 0;
//        double turretAngle = turretAngleAnalog.getVoltage();
//
//        double turretPower;
//        if (turretStick != 0) {
//            if (turretHolding) {
//                turretPID.reset();
//                turretHolding = false;
//            }
//            turretPower = turretStick;
//        } else {
//            if (!turretHolding) {
//                turretTargetPos = turretRotations;
//                turretHolding = true;
//            }
//            turretPower = turretPID.run(turretTargetPos, turretRotations);
//        }
////        should we put autoAim in the autopath class?
////        if (autoAim) { //Auto-aim for auto
////            Point target = new Point(GOAL_X, GOAL_Y);
////            double angleToTarget = Math.atan2(target.x - Robot.getRobotPose().getX(), target.y - Robot.getRobotPose().getY());
////            double currentAngle = Robot.getRobotPose().getHeading() + turretAngle;
////            turretPower = turretPID.run(angleToTarget, currentAngle);
////        }
//        turret.setPower(turretStick);
//
//        //Linkage
//        double elapsed = linkageTimer.milliseconds();
//        double targetPos = linkageOne.getPosition() + LINKAGE_STICK_COEF * elapsed * linkageStick;
//        linkageOne.setPosition(Range.clip(targetPos, LINKAGE_MIN_POS, LINKAGE_MAX_POS));
//        linkageTimer.reset();
//
//
//        //Hopper
//        if (releaseButton && !this.releaseButton) {
//            this.releaseButton = true;
//            releaseOpen = !releaseOpen;
//        } else if (!releaseButton && this.releaseButton) {
//            this.releaseButton = false;
//        }
//
//        if (releaseOpen) {
//            release.setPosition(releasePosOpen);
//        } else {
//            release.setPosition(releasePosClosed);
//        }
//
//
//        //Run To Position
//        if (resetButton) {
//            linkageOne.setPosition(BASE_LINKAGE_POS);
//            liftToPosition(BASE_LIFT_POS);
//            turretToPosition(BASE_TURRET_POS);
//        }
//    }

    LiftState liftState = LiftState.HOME;
    boolean liftRunning = false;
    boolean turretRunning = false;
    boolean liftReady = false;
    public void basicRun(boolean shared, boolean alliance, boolean reset, double liftAdjust, double turretAdjust, boolean linkageButton, boolean releaseButton) {
        double liftPos = lift.getCurrentPosition();
        double turretPos = turret.getCurrentPosition();

        //Linkage Code
        if(linkageButton && !this.linkageButton){
            this.linkageButton = true;
            linkageOpen = !linkageOpen;
        } else if (!linkageButton && this.linkageButton){
            this.releaseButton = false;
        }

        if(linkageOpen){
            linkageOne.setPosition(LINKAGE_MAX_POS);
            linkageTwo.setPosition(LINKAGE_MAX_POS);
        } else {
            linkageOne.setPosition(LINKAGE_MIN_POS);
            linkageTwo.setPosition(LINKAGE_MAX_POS);
        }

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

        LiftState prevLiftState = liftState;
        if(shared) {
            liftState = LiftState.SHARED;
        } else if(alliance) {
            liftState = LiftState.ALLIANCE;
        } else if(reset) {
            liftState = LiftState.HOME;
        }
        if(liftAdjust != 0 || turretAdjust != 0) {
           liftState = LiftState.MANUAL;
        }
        
        if(prevLiftState != liftState) {
            turretRunning = false;
            liftRunning = false;
        }

        boolean turretClosed = Math.abs(turretPos) < 35;
        boolean liftCleared = liftPos > LIFT_CLEARED_POS;
        boolean liftUpAlliance = liftPos > LIFT_ALLIANCE_POS;

        //state machine controller
        switch (liftState) {
            case HOME:
                linkageOne.setPosition(0.1);
                linkageTwo.setPosition(0.1);
                if(!turretClosed && !liftCleared && !liftRunning) {
                    liftRunning = true;
                    liftToPosition(LIFT_CLEARED_POS + 10);
                } else if(!turretClosed && liftCleared && liftRunning && !turretRunning) {
                    // liftrunning false, and run turret
                    liftRunning = false;
                    turretToPosition(0);
                    turretRunning = true;
                } else if(turretClosed) {
                    if(turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        turretToPosition(0);
                    }
                    liftToPosition(0);
                    lift.setPower(0.5);
                    liftRunning = false;
                    turretRunning = false;
                }

                break;
            case SHARED:
                if(turretClosed && !liftCleared && !liftRunning){
                    liftRunning = true;
                    liftToPosition(LIFT_CLEARED_POS + 10, 0.3);
                } else if(liftCleared){
                    liftRunning = false;
                    turretToPosition(turret90Degrees, 0.3);
                    turretRunning = true;
                } else if(Math.abs(turretPos) > turret90Degrees - 10){
                    turretRunning = false;
                }

                break;
            case ALLIANCE:
                if(turretClosed && !liftUpAlliance && !liftRunning){
                    liftRunning = true;
                    liftToPosition(LIFT_ALLIANCE_POS + 10, 0.3);
                } else if(liftCleared){
                    liftRunning = false;
                    turretToPosition(turret90Degrees, 0.3);
                    turretRunning = true;
                } else if(Math.abs(turretPos) > turret90Degrees - 10){
                    turretRunning = false;
                }

                break;
            case MANUAL:
                liftRunning = false;
                turretRunning = false;
                if(lift.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    lift.setPower(0.0);
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if(turret.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    turret.setPower(0.0);
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                double liftPower;
                int minPos = (turretClosed) ? 0 : LIFT_CLEARED_POS;
                if (liftAdjust != 0) {
                    if (liftHolding) {
                        liftHolding = false;
                        liftPID.reset();
                    }
                    if (liftAdjust > 0) {
                        liftPower = liftAdjust * sigmoid(LIFT_CONVERGENCE_SPEED * (liftPos - (LIFT_ALLIANCE_POS - LIFT_LIMIT_RANGE)));
                    } else {
                        liftPower = liftAdjust * sigmoid(LIFT_CONVERGENCE_SPEED * (LIFT_LIMIT_RANGE / 2 - (liftPos - minPos)));
                    }
                } else {
                    if (!liftHolding) {
                        liftTargetPos = Range.clip(liftPos, minPos, LIFT_MAX_POS);
                        liftHolding = true;
                    }
                    liftPower = liftPID.run(liftTargetPos, liftPos);
                }

                lift.setPower(liftPower);
                turret.setPower(turretAdjust);
                break;
        }

        if(Math.abs(turretPos) < 30 && Math.abs(liftPos) < 20) {
            liftReady = true;
        } else {
            liftReady = false;
        }

        telemetry.addData("turret pos", turretPos);
        telemetry.addData("lift pos", liftPos);
        telemetry.addData("lift current", lift.getCurrent(CurrentUnit.AMPS));

    }

    enum LiftState {
        SHARED,
        ALLIANCE,
        HOME,
        MANUAL
    }



    public void turretToPosition(int pos){
        turret.setTargetPosition(pos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }

    public void turretToPosition(int pos, double power){
        turret.setTargetPosition(pos);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(power);
    }

    public void liftToPosition(int pos) {
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
    }
    public void liftToPosition(int pos, double power) {
        lift.setTargetPosition(pos);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(power);
    }

    public boolean isReset() {
        return liftReady;
    }

    public void test(double liftStick, double turretStick, boolean linkageButton, boolean releaseButton) {

        double liftPower = liftStick;
        double liftFeedForward = 0.15;
//        if(lift.getCurrentPosition() > 50) {
//            liftPower += liftFeedForward;
//        }
        lift.setPower(liftPower);
        turret.setPower(turretStick);
        if(linkageButton) {
            linkageOne.setPosition(0.8);
            linkageTwo.setPosition(0.8);
        } else {
            linkageOne.setPosition(0.1);
            linkageTwo.setPosition(0.1);
        }

        if(releaseButton) {
            release.setPosition(0.23);
        } else {
            release.setPosition(0.6);
        }

        telemetry.addData("lift pos", lift.getCurrentPosition());
        telemetry.addData("turret pos", turret.getCurrentPosition());
        telemetry.addData("lift power", liftStick);
    }

    public static double sigmoid(double x) {
        return 1 / (1 + Math.exp(-x));
    }

    private class LiftMin implements UnivariateFunction {
        public double value(double turretAngle) {
            double minHeight;
            if(turretAngle > -Math.PI/12 && turretAngle < Math.PI/12) {
                minHeight = 0;
            } else {
                minHeight = Lift.this.LIFT_MAX_POS;
            }
            return minHeight;
        }
    }

    private class ExtensionMax implements UnivariateFunction {
        public double value(double turretAngle){
            double extend;
            if(turretAngle > -Math.PI/2 && turretAngle < Math.PI/2) {
                extend = 1;
            } else {
                extend = 0;
            }
            return extend;
        }
    }
}