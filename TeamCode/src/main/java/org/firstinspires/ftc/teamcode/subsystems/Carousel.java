package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Carousel extends Subsystem {
    private CRServo carousel, carouselTwo;


    @Override
    public void init() {
        carousel = hardwareMap.crservo.get("carousel_one");
        carouselTwo = hardwareMap.crservo.get("carousel_two");
    }

    public void run(boolean spin) {
        if(spin) {
            carousel.setPower(-1.0);
            carouselTwo.setPower(1.0);
        } else{
            carousel.setPower(0);
            carouselTwo.setPower(0);
        }


    }

    public void setCarouselPower(double power) {
        carousel.setPower(power);
        carouselTwo.setPower(power);
    }

}
