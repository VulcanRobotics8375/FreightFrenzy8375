package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.HardwarePrecision;

public class Lift extends Subsystem {

    private Servo extend;
    private DcMotor lift;

    private boolean extended;
    private boolean extendButton;
    private double liftPower;

    @Override
    public void init() {
        lift = hardwareMap.dcMotor.get("lift");
        extend = hardwareMap.servo.get("extend");

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void run(double liftPower, boolean extend) {
        if(extend && !extendButton) {
            extended = !extended;
            extendButton = true;
        }
        if(!extend && extendButton) {
            extendButton = false;
        }

        if(extended) {
            this.extend.setPosition(0.01);
        } else {
            this.extend.setPosition(1);
        }
        if(MathUtils.shouldHardwareUpdate(liftPower, this.liftPower, HardwarePrecision.HIGH)) {
            lift.setPower(liftPower);
        }

        this.liftPower = liftPower;
    }

}
