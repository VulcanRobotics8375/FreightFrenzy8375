package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Cap extends Subsystem {
    private Servo arm;
    private double down = 0.7;
    private double up = 0.01;
    private boolean open = false;
    boolean currentButton = false;

    @Override
    public void init() {
        arm = hardwareMap.servo.get("cap_arm");
        arm.setPosition(up);
    }

    public void run(boolean changeOpen) {

       if(currentButton==changeOpen)
           return;
        if (changeOpen && !open) {
            arm.setPosition(up);
            open = true;
        }
        else if (changeOpen && open) {
            arm.setPosition(down);
            open = false;
        }
        currentButton = changeOpen;
    }



}
