package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Cap extends Subsystem {
    private Servo arm;

    ElapsedTime capTimer = new ElapsedTime();
    private final double downPos = 0.05;
    private final double upPos = 0.65;
    private boolean goingUp = false;
    private double position = upPos;
    private final double ARM_COEF = 0.002;

    @Override
    public void init() {
        arm = hardwareMap.servo.get("cap_arm");
        arm.setDirection(Servo.Direction.REVERSE);
        arm.setPosition(upPos);
    }

    public void run(double down, double up) {
        if(up > 0 && down == 0) {
            setPower(up);
            goingUp = true;
        } else if(down > 0 && up == 0) {
            setPower(-down);
            goingUp = false;
        } else if(goingUp) {
            setPower(-down);
        } else {
            setPower(up);
        }
    }

    public void setPower(double power) {
        double elapsed = capTimer.milliseconds();
        double targetPos = Range.clip(arm.getPosition() + power * elapsed * ARM_COEF, downPos, upPos);
        arm.setPosition(targetPos);
        capTimer.reset();
    }
}
