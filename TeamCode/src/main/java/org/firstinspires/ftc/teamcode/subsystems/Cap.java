package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Cap extends Subsystem {
    private Servo arm;
    private double down = 0.45;
    private double up = 0.01;
    private double open = -1;
    private boolean changeOpen = false;

    @Override
    public void init() {
        arm = hardwareMap.servo.get("cap_arm");
        arm.setPosition(up);
    }

    public void run(boolean changeOpen) {
       if(changeOpen && !this.changeOpen) {
           open *= -1;
           this.changeOpen = true;
       }
       if(!changeOpen && this.changeOpen){
           this.changeOpen = false;
       }
       if(open > 0){
           arm.setPosition(down);
       }
       if(open < 0){
           arm.setPosition(up);
       }
    }



}
