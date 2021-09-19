package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

@SuppressWarnings("FieldCanBeLocal")
public class Carousel extends Subsystem {
    private CRServo carousel;

    private final double carouselPower = 1.0;

    public void init(){
        carousel = hardwareMap.crservo.get("duck_servo");
    }

    public void run(boolean buttonOn){
        if(buttonOn){
            carousel.setPower(carouselPower);
        }else {
            carousel.setPower(0);
        }
    }

}
