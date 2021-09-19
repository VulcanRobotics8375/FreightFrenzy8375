package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robotcorelib.util.JoystickCurve;
import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.util.JoystickCurve;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Drivetrain extends Subsystem {

    private DcMotor fl, fr, bl, br;
    public static final DriveMode driveMode = DriveMode.TANK;


    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("front_left");
        fr = hardwareMap.dcMotor.get("front_right");
        bl = hardwareMap.dcMotor.get("back_left");
        br = hardwareMap.dcMotor.get("back_right");

        setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

    public void setPowers(double fl, double fr, double bl, double br) {
        this.fl.setPower(fl);
        this.fr.setPower(fr);
        this.bl.setPower(bl);
        this.br.setPower(br);
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
        double theta = Math.atan2(forward, strafe) - Math.PI / 4.0;
        forward = MathUtils.joystickCurve(forward, JoystickCurve.MODIFIED_CUBIC);
        strafe = MathUtils.joystickCurve(strafe, JoystickCurve.MODIFIED_CUBIC);
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

    public void setPowers(double[] powers) {
        this.fl.setPower(powers[0]);
        this.fr.setPower(powers[1]);
        this.bl.setPower(powers[2]);
        this.br.setPower(powers[3]);

    }

    public void setDrivetrainMode(DcMotor.RunMode runMode) {
        fl.setMode(runMode);
        fr.setMode(runMode);
        bl.setMode(runMode);
        br.setMode(runMode);
    }
}

enum DriveMode {
    TANK,
    MECANUM

}
