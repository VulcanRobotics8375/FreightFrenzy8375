package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Carousel extends Subsystem {
    private CRServo carousel;
    private Servo opener;

    private double CarouselToggle = -1;
    private boolean CarouselButton = false;
    private double close = 0.3;
    private double open = 0.6;

    @Override
    public void init() {
        carousel = hardwareMap.crservo.get("carousel");
        opener = hardwareMap.servo.get("carousel_opener");
    }

    public void run(boolean spin, boolean changeOpen) {
        if(spin) {
            carousel.setPower(1);
        }

        if(changeOpen && !CarouselButton) {
            CarouselToggle *=-1;
            CarouselButton = true;
        } else if(!changeOpen && CarouselButton) {
            CarouselButton = false;
        }

        if(CarouselToggle==1)  opener.setPosition(open);
        else if(CarouselToggle ==-1) opener.setPosition(close);


    }
}
