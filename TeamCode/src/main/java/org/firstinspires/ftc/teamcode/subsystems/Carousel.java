package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Carousel extends Subsystem {
    private CRServo carousel;
    private Servo opener;

    private boolean opened = false;
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

        if(changeOpen && !opened) {
            opener.setPosition(open);
        } else if(changeOpen && opened) {
            opener.setPosition(close);
        }
    }
}
