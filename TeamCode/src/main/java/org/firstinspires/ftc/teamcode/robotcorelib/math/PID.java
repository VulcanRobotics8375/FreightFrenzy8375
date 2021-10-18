package org.firstinspires.ftc.teamcode.robotcorelib.math;

import java.util.*;
import java.io.*;

public class PID {
    private double Kp;
    private double Ki;
    private double Kd;
    private double error;
    private double lastError;
    private double integral;
    private double derivative;

    public PID(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double run(double target, double measurement){
        error = target - measurement;
        if(Math.abs(integral) > Math.abs(error) * Kp) {
            integral = 0;
        }
        integral += (error + lastError) / 2.0;
        derivative = error - lastError;
        lastError = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }
}
