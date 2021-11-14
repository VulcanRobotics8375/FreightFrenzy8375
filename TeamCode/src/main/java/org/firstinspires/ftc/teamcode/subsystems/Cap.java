package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Cap extends Subsystem {
    private Servo arm;
    private double down = 0.7;
    private double up = 0.1;

    @Override
    public void init() {
        arm = hardwareMap.servo.get("cap_arm");
        arm.setPosition(up);
    }

    public void run(boolean changePos) {
        if (changePos && arm.getPosition() == down) arm.setPosition(up);
        else if (changePos && arm.getPosition() == up) arm.setPosition(down);
    }



}
