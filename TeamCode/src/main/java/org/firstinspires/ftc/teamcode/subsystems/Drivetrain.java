package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robotcorelib.drive.DriveMode;
import org.firstinspires.ftc.teamcode.robotcorelib.drive.DrivetrainImpl;
import org.firstinspires.ftc.teamcode.robotcorelib.util.JoystickCurve;
import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.HardwarePrecision;

public class Drivetrain extends Subsystem implements DrivetrainImpl {

    private DcMotor fl, fr, bl, br;
    private BNO055IMU imu;
    public static final DriveMode driveMode = DriveMode.MECANUM;


    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    private CRServo odometryLift;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("front_left");
        fr = hardwareMap.dcMotor.get("front_right");
        bl = hardwareMap.dcMotor.get("back_left");
        br = hardwareMap.dcMotor.get("back_right");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        odometryLift = hardwareMap.crservo.get("odometry_lift");

        setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //set up IMU
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        if(Drivetrain.driveMode == DriveMode.TANK) {
            fl.setDirection(DcMotorSimple.Direction.REVERSE);
            fr.setDirection(DcMotorSimple.Direction.FORWARD);
            bl.setDirection(DcMotorSimple.Direction.REVERSE);
            br.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if(Drivetrain.driveMode == DriveMode.MECANUM) {
            fl.setDirection(DcMotorSimple.Direction.FORWARD);
            fr.setDirection(DcMotorSimple.Direction.REVERSE);
            bl.setDirection(DcMotorSimple.Direction.FORWARD);
            br.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void tankDrive(double forward, double turn) {
        forward = MathUtils.joystickCurve(forward, JoystickCurve.MODIFIED_CUBIC);
        turn = MathUtils.joystickCurve(turn, JoystickCurve.MODIFIED_CUBIC);
         double magnitude = Math.abs(forward) + Math.abs(turn);
         if(magnitude > 1) {
             forward *= 1 / magnitude;
             turn *= 1 / magnitude;
         }
        setPowers(forward+turn, forward-turn, forward+turn, forward-turn);
    }

    public void mechanumDrive(double forward, double strafe, double turn) {
        double multiplier = 2.0 / Math.sqrt(2.0);
        forward = MathUtils.joystickCurve(forward, JoystickCurve.MODIFIED_CUBIC);
        strafe = MathUtils.joystickCurve(strafe, JoystickCurve.MODIFIED_CUBIC);

        double theta = Math.atan2(forward, strafe) - Math.PI / 4.0;
//        turn = MathUtils.joystickCurve(turn, JoystickCurve.MODIFIED_CUBIC);

        double magnitude = Math.abs(forward) + Math.abs(strafe) + Math.abs(turn);
        if(magnitude > 1) {
            forward *= 1 / magnitude;
            strafe *= 1 / magnitude;
            turn *= 1 / magnitude;
        }

        // Godly Math Trick: sin(x+pi/4) = cos(x-pi/4)
        double speed = multiplier * Math.hypot(strafe, forward);

        double flSpeed = speed * Math.sin(theta) + turn;
        double frSpeed = speed * Math.cos(theta) - turn;
        double blSpeed = speed * Math.cos(theta) + turn;
        double brSpeed = speed * Math.sin(theta) - turn;


        setPowers(flSpeed, frSpeed, blSpeed, brSpeed);
    }

    @Override
    public void setPowers(double[] powers) {
        DcMotor[] motors = getMotorsAsList();
        for (int i = 0; i < 4; i++) {
            if(MathUtils.shouldHardwareUpdate(powers[i], motors[i].getPower(), HardwarePrecision.HIGH)) {
                motors[i].setPower(powers[i]);
            }
        }
    }

    public void setPowers(double fl, double fr, double bl, double br) {
        setPowers(new double[] { fl, fr, bl, br });
    }

    public void startOdometryLift() {
        odometryLift.setPower(-1.0);
    }

    public void stopOdometryLift() {
        odometryLift.setPower(0.0);
    }

    public void setDrivetrainMode(DcMotor.RunMode runMode) {
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
    }

    public DcMotor[] getMotorsAsList() {
        return new DcMotor[] { fl, fr, bl, br };
    }

    public BNO055IMU getIMU() {
        return imu;
    }

    @Override
    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        fl.setZeroPowerBehavior(zeroPowerBehavior);
        fr.setZeroPowerBehavior(zeroPowerBehavior);
        bl.setZeroPowerBehavior(zeroPowerBehavior);
        br.setZeroPowerBehavior(zeroPowerBehavior);
    }

}

