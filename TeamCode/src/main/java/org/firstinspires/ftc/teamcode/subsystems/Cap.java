package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
// import java.time.*;

public class Cap extends Subsystem {
    private Servo arm;
    private double down = 0.01;
    private double up = 0.65;
    private double open = -1;
    private boolean changeOpen = false;

    private double position = up;
    private double increment = 0.005;
    long lastTime = System.nanoTime();

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
           // updated to slow down so cap doesn't fly out
           if(position - increment > down && System.nanoTime() - lastTime
                   > 15000000) {
               arm.setPosition(position-increment);
               position -= increment;
               lastTime = System.nanoTime();
           }
       }
//       if(open > 0){
//           // updated to slow down so cap doesn't fly out
//           if(position + increment <= 0.6 && System.nanoTime() - lastTime
//                > 10000000) {
//               arm.setPosition(position+increment);
//               position += increment;
//               lastTime = System.nanoTime();
//           } else if(position + increment > 0.6 && System.nanoTime() - lastTime
//                   > 10000000) {
//               arm.setPosition(down);
//               position = 0.6;
//               lastTime = System.nanoTime();
//           }
//
//       }
       if(open < 0){
           arm.setPosition(up);
           position = up;
       }
    }

    public void setArmPosition(double pos) {
        arm.setPosition(pos);
    }



}
