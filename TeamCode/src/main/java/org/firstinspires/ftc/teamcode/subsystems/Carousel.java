package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Carousel extends Subsystem {
    private CRServo carousel;
    private Servo opener;

    private boolean openButton = false;
    private double openOn = -1;

    @Override
    public void init() {
        carousel = hardwareMap.crservo.get("carousel");
        opener = hardwareMap.servo.get("carousel_opener");
    }

    public void run(boolean spin, boolean openButton, boolean oppositeSpin) {
        if(spin) {
            carousel.setPower(-1);
        } else{
            carousel.setPower(0);
        }

        if(oppositeSpin){
            carousel.setPower(1);
        } else{
            carousel.setPower(0);
        }

        if (openButton && !this.openButton) {
            openOn *= -1;
            this.openButton = true;
        }
        if (!openButton && this.openButton) {
            this.openButton = false;
        }
        if(openOn > 0){
            opener.setPosition(0.01);
        }
        if(openOn < 0){
            opener.setPosition(1.0);
        }
    }

    public void setOpenerPosition(double pos) {
        opener.setPosition(pos);
    }

    public void setCarouselPower(double power) {
        carousel.setPower(power);
    }

}
