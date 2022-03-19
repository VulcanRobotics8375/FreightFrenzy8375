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
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.math.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.util.JoystickCurve;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Lift extends Subsystem {
    private DcMotorEx lift;
    private DcMotorEx turret;
    private Servo release;
    private Servo linkageOne, linkageTwo;

    SimplePID liftPID = new SimplePID(0.005, 0.0, 0.0, -1.0, 1.0);
    private double liftTargetPos;
    private boolean liftHolding = false;
    private LiftMin LIFT_MIN = new LiftMin();
    private final int LIFT_MAX_POS = 570;
    private final int LIFT_CLEARED_POS = 380;
    private final int LIFT_ALLIANCE_POS = 510;
    private final double LIFT_CONVERGENCE_SPEED = 0.1;
    private final double LIFT_LIMIT_RANGE = 100.0;

    private boolean auto = false;
    private boolean flipped = false;

    private LinkageForwardKinematics linkageForwardKinematics = new LinkageForwardKinematics();
    private PolynomialSplineFunction inverseLinkageKinematics;
    private ElapsedTime linkageTimer = new ElapsedTime();
    private boolean linkageButton = false;
    private boolean linkageOpen = false;
    private double linkagePower;
    private final double LINKAGE_MIN_POS = 0.1;
    private final double LINKAGE_MAX_POS = 0.86;
    private final double LINKAGE_MIN_INCHES = linkageForwardKinematics.value(LINKAGE_MIN_POS);
    private final double LINKAGE_MAX_INCHES = linkageForwardKinematics.value(LINKAGE_MAX_POS);
    private final double LINKAGE_POWER_COEF = 0.02;
    private final double ANALOG_ENCODER_VOLTAGE_OFFSET = 1.37;

    private final double TURRET_TICKS_PER_DEGREE = 1456.0 / 360.0;
    private final double TURRET_VOLTS_PER_DEGREE = (3.3 * 5.0) / 360.0;
    private final double TURRET_TICKS_PER_VOLT = TURRET_TICKS_PER_DEGREE / TURRET_VOLTS_PER_DEGREE;
    private double turretOffset;

    private boolean releaseOpen = false;
    private boolean releaseButton = false;
    private final double releasePosOpen = 0.23;
    private final double releasePosClosed = 0.4;

    private AnalogInput turretAngleAnalog; //Temporary until analog input
    private final int turret90Degrees = -390;

    public void init() {

        //inverse linkage kinematics: cubic spline estimation of inverse curve
        double[] x = new double[] {
                0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 23.6
        };
        double[] y = new double[] {
                0.1,
                0.24829894162,
                0.341717448755,
                0.413980186568,
                0.474883421125,
                0.529068086363,
                0.579273889518,
                0.627418731134,
                0.675115801525,
                0.724065058856,
                0.77661899685,
                0.837325861947,
                0.9
        };
        SplineInterpolator interpolator = new SplineInterpolator();
        inverseLinkageKinematics = interpolator.interpolate(x, y);

        //Hardware Mapping
        release = hardwareMap.servo.get("release");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        linkageOne = hardwareMap.servo.get("linkage_one");
        linkageTwo = hardwareMap.servo.get("linkage_two");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turretAngleAnalog = hardwareMap.get(AnalogInput.class, "turret_encoder");

        //motor and servo modes
        //lift
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        //turret
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        //servo modes
        linkageTwo.setDirection(Servo.Direction.REVERSE);

        //calculate turet offset based on absolute encoder voltage
        turretOffset = (TURRET_TICKS_PER_VOLT * (turretAngleAnalog.getVoltage() - ANALOG_ENCODER_VOLTAGE_OFFSET));
        PIDFCoefficients stockTurretCoeff = turret.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//        turret.setVelocityPIDFCoefficients(stockTurretCoeff.p, stockTurretCoeff.i, stockTurretCoeff.d + 0.02, stockTurretCoeff.f);
//        lift.setVelocityPIDFCoefficients(stockLiftCoeff.p, stockLiftCoeff.i, stockLiftCoeff.d + 0.01, stockLiftCoeff.f);
        lift.setPositionPIDFCoefficients(12.0);
        turret.setPositionPIDFCoefficients(19.0);
    }

    //independent state machine variables
    //there is a main state machine, liftState, that controls most of the lift.
    // Some components of the lift are state-independent or need slightly more complex logic,
    // so we add some booleans to control those states.
    LiftState liftState = LiftState.HOME;
    boolean liftRunning = false;
    boolean turretRunning = false;
    boolean liftReady = false;
    boolean flippingSides = false;
    LiftState linkageAdjustState = LiftState.SHARED;
    boolean linkageGoingUp = false;
    double linkageAdjustAmountInches = 0.0;
    boolean relocalize = false;
    boolean automaticLinkageExtend;
    double linkagePos = LINKAGE_MAX_POS;
    //main teleop arm and turret sequence
    public void runTurretAndArm(boolean shared, boolean alliance, boolean reset, double liftAdjust, double turretAdjust, boolean linkageButton, boolean releaseButton, double linkageForward, double linkageBack, boolean flipSides) {
        double liftPos = lift.getCurrentPosition();
        double turretPos = turret.getCurrentPosition() + turretOffset;

        if(flipSides && !flippingSides) {
            flippingSides = true;
            flipped = !flipped;
        }
        if(!flipSides && flippingSides) {
            flippingSides = false;
        }

        int turret90Degrees = (this.turret90Degrees + ((flipped ? 1 : -1)*(int)turretOffset)) * (flipped ? -1 : 1);
        int teleopAlliancePos;
        if(auto) {
            teleopAlliancePos = turret90Degrees;
        } else {
            teleopAlliancePos = (this.turret90Degrees + (int)turretOffset - 200) * (flipped ? -1 : 1);
        }

        //Linkage Code
        if(linkageButton && !this.linkageButton){
            this.linkageButton = true;
            linkageOpen = !linkageOpen;
        } else if (!linkageButton && this.linkageButton){
            this.linkageButton = false;
        }

        if(linkageForward > 0 && linkageBack == 0) {
            linkagePower = linkageForward;
            linkageGoingUp = true;
        } else if(linkageBack > 0 && linkageForward == 0) {
            linkagePower = -linkageBack;
            linkageGoingUp = false;
        } else if(linkageGoingUp) {
            linkagePower = -linkageBack;
        } else {
            linkagePower = linkageForward;
        }

        if(linkageOpen) {
            linkageAdjustAmountInches += linkageForwardKinematics.value(linkagePos + linkagePower * LINKAGE_POWER_COEF) - linkageForwardKinematics.value(linkagePos);
            if (linkagePos + linkageAdjustAmountInches > LINKAGE_MAX_INCHES) {
                linkageAdjustAmountInches = LINKAGE_MAX_INCHES - linkagePos;
            } else if (linkagePos + linkageAdjustAmountInches < LINKAGE_MIN_INCHES) {
                linkageAdjustAmountInches = LINKAGE_MIN_INCHES - linkagePos;
            }
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
        } else if(reset && !linkageOpen) {
            liftState = LiftState.HOME;
        }
        if(liftAdjust != 0 || turretAdjust != 0) {
            liftState = LiftState.MANUAL;
        }

        if(prevLiftState != liftState) {
            liftTargetPos = liftPos;
            turretRunning = false;
            liftRunning = false;
            automaticLinkageExtend = false;
        }

        boolean turretClosed = Math.abs(turretPos) < 35;
        boolean liftCleared;

        //state machine controller
        boolean turretOut = Math.abs(turretPos) > Math.abs(turret90Degrees);
        switch (liftState) {
            case HOME:
                releaseOpen = false;
                liftCleared = liftPos > LIFT_CLEARED_POS || turretClosed;
                linkageOpen = false;
                if(!liftCleared) {
                    liftToPosition(LIFT_CLEARED_POS + 10, 1.0);
                } else if(!turretClosed) {
                    turretToPosition(-(int)turretOffset, 1.0);
                } else {
                    turretToPosition(-(int)turretOffset, 0.5);
                    liftToPosition(2, 0.5);
                }
                break;
            case SHARED:
                if(!(linkageAdjustState == LiftState.SHARED)) {
                    linkageAdjustAmountInches = 0.0;
                    linkageAdjustState = LiftState.SHARED;
                }

                linkagePos = 0.4;
                liftCleared = liftPos > LIFT_CLEARED_POS;
                if(linkageOpen && !turretClosed) {
                    liftToPosition(100, 1.0);
                } else {
                    liftToPosition(LIFT_CLEARED_POS + 10, 1.0);
                }
                if(liftCleared || turretOut) {
                    turretToPosition(turret90Degrees);
                }
                break;
            case ALLIANCE:
                if(!(linkageAdjustState == LiftState.ALLIANCE)) {
                    linkageAdjustAmountInches = 0.0;
                    linkageAdjustState = LiftState.ALLIANCE;
                }

                if(prevLiftState != LiftState.ALLIANCE) {
                    linkagePos = LINKAGE_MAX_POS;
                }
                liftCleared = liftPos > LIFT_CLEARED_POS;
                liftToPosition(LIFT_ALLIANCE_POS, 1.0);
                if(liftCleared && Math.abs(turretPos) >= Math.abs(turret90Degrees / 2.0) && !automaticLinkageExtend) {
                    linkageOpen = true;
                    automaticLinkageExtend = true;
                }
                if(liftCleared || turretOut) {
                    turretToPosition(teleopAlliancePos, 1.0);
                }
                break;
            case MANUAL:
                if(lift.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    lift.setPower(0.0);
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if(turret.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                    turret.setPower(0.0);
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                double liftPower;
                int minPos;
                if(turretClosed) {
                    minPos = 0;
                } else if(linkageOpen) {
                    minPos = 100;
                } else {
                    minPos = LIFT_CLEARED_POS;
                }

                if (Math.abs(liftAdjust) > 0.05) {
                    if (liftHolding) {
                        liftHolding = false;
                        liftPID.reset();
                    }
                    if (liftAdjust > 0) {
                        if(liftPos > LIFT_MAX_POS - LIFT_LIMIT_RANGE) {
                            liftPower = liftAdjust - ((liftAdjust / LIFT_LIMIT_RANGE) * (liftPos - (LIFT_MAX_POS - LIFT_LIMIT_RANGE)));
                        } else {
                            liftPower = liftAdjust;
                        }
                    } else {
                        if(liftPos < minPos + LIFT_LIMIT_RANGE) {
                            liftPower = (liftAdjust / LIFT_LIMIT_RANGE) * (liftPos - minPos);
                        } else {
                            liftPower = liftAdjust;
                        }
                    }
                } else {
                    if (!liftHolding) {
                        liftTargetPos = Range.clip(liftPos, minPos, LIFT_MAX_POS);
                        liftHolding = true;
                    }
                    double feedForward = 0.03;
                    liftPower = liftPID.run(liftTargetPos, liftPos) + feedForward;
                }

                telemetry.addData("lift PID feedforward", lift.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f);
                telemetry.addData("lift power", liftPower);
                telemetry.addData("lift pos", liftPos);
                telemetry.addData("lift current", lift.getCurrent(CurrentUnit.AMPS));
//                telemetry.addData("lift holding", liftHolding);
                if(lift.getCurrent(CurrentUnit.AMPS) > 14) {
                    liftPower *= 0.8;
                }
                lift.setPower(liftPower);

                double turretPower = MathUtils.joystickCurve(turretAdjust, JoystickCurve.MODIFIED_CUBIC) * (linkageOpen ? 0.2 : 1.0);
                turret.setPower(turretPower);
                break;
        }

        if(linkageOpen){
            double linkageTargetInches = Range.clip(linkageForwardKinematics.value(linkagePos) + linkageAdjustAmountInches, 0.0, 23.6);
            double linkageTargetPos = inverseLinkageKinematics.value(linkageTargetInches);
            linkageTargetPos = Range.clip(linkageTargetPos, LINKAGE_MIN_POS, LINKAGE_MAX_POS);
            setLinkagePos(linkageTargetPos);
        } else {
            setLinkagePos(LINKAGE_MIN_POS);
        }

        if(!liftReady) {
            relocalize = false;
        }
        if(liftReady && !relocalize) {
            relocalize = true;
            turretOffset += (TURRET_TICKS_PER_VOLT * (turretAngleAnalog.getVoltage() - ANALOG_ENCODER_VOLTAGE_OFFSET));
        }
        liftReady = Math.abs(turretPos) < 30 && Math.abs(liftPos) < 10;

//        telemetry.addData("lift state", liftState.toString());
//        telemetry.addData("analog encoder", turretAngleAnalog.getVoltage());
//        telemetry.addData("turret pos", turretPos);
//        telemetry.addData("turret pos raw", turret.getCurrentPosition());
//        telemetry.addData("lift pos", liftPos);
//        telemetry.addData("lift current", lift.getCurrent(CurrentUnit.AMPS));

    }

    public void runTurretAndArm() {
        runTurretAndArm(false, false, false, 0.0, 0.0, false, false, 0.0, 0.0, false);
    }

    public void runNoLimits(double liftPower, double turretPower) {
        lift.setPower(liftPower);
        turret.setPower(turretPower);
    }

    public boolean isLinkageOpen() {
        return linkageOpen;
    }

    public void reset() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretOffset = (TURRET_TICKS_PER_VOLT * (turretAngleAnalog.getVoltage() - ANALOG_ENCODER_VOLTAGE_OFFSET));
    }

    enum LiftState {
        SHARED,
        ALLIANCE,
        HOME,
        MANUAL
    }


    public int getLiftPosition() {
        return lift.getCurrentPosition();
    }

    public double getTurretPosition() {
        return turret.getCurrentPosition() + turretOffset;
    }

    public int getLiftAlliancePos() { return LIFT_ALLIANCE_POS; }
    public int getLiftSharedPos() {
        return LIFT_CLEARED_POS;
    }

    public int getTurret90Degrees() { return this.turret90Degrees; }

    public double getTurretOffset() { return turretOffset; }

    public void turretToPosition(int pos){
        turret.setTargetPosition(pos);
        if(turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        turret.setPower(1);
    }

    public void turretToPosition(int pos, double power){
        turret.setTargetPosition(pos);
        if(turret.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        turret.setPower(power);
    }

    public void autoMode() {
        auto = true;
    }

    public void liftToPosition(int pos) {
        if(lift.getTargetPosition() != pos) {
            lift.setTargetPosition(pos);
        }
        if(lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        lift.setPower(1);
    }
    public void liftToPosition(int pos, double power) {
        lift.setTargetPosition(pos);
        if(lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        lift.setPower(power);
    }

    public boolean isReset() {
        return liftReady;
    }

    public void setLinkagePos(double pos) {
        linkageOne.setPosition(pos);
        linkageTwo.setPosition(pos);
    }

    public void setGlobalLinkagePos(double pos) {
        linkagePos = pos;
    }
    public void setReleasePosition(double pos) {
        release.setPosition(pos);
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

    static class LinkageForwardKinematics implements UnivariateFunction {
        @Override
        public double value(double x) {
            double servoPosToTheta = (((3 - (Math.PI / 12.0)) / 0.9) * (x - 0.1)) + (Math.PI / 12.0);
            return -3.0 * (4.235 * Math.cos(servoPosToTheta) + (7.7 * Math.cos(Math.PI + Math.asin(((4.235 * Math.sin(servoPosToTheta) - 0.933) / 7.7)))) + 3.60757658307);
        }
    }

}