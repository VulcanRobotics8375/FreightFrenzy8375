package org.firstinspires.ftc.teamcode.robotcorelib.math;

import java.util.*;
import java.io.*;

public class PID {
    private double Kp;
    private double Ki;
    private double Kd;
    private double outputMaxLimit;
    private double outputMinLimit;
    private double error;
    private double lastError;
    private double integral;
    private double derivative;
    private double controllerOutput;
    private double controllerOutputB;
    private boolean isSaturating = false;
    private boolean integratorSaturate = false;

    public PID(double Kp, double Ki, double Kd, double outputMaxLimit, double outputMinLimit){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.outputMaxLimit = outputMaxLimit;
        this.outputMinLimit = outputMinLimit;
    }

    public double run(double target, double measurement){
        error = target - measurement;
        integral += (error + lastError) / 2.0;
        derivative = error - lastError;
        lastError = error;

        // Matt Attempt at Integral AntiWindup: Clamping
        controllerOutput = Kp * error + Ki * integral + Kd * derivative;
        controllerOutputB = controllerOutput;
        if(controllerOutput > outputMaxLimit){
            controllerOutput = outputMaxLimit;
        } else if(controllerOutput < outputMinLimit){
            controllerOutput = outputMinLimit;
        }

        if(controllerOutputB == controllerOutput){
            isSaturating = false;
        }else{
            isSaturating = true;
        }

        if((controllerOutputB > 0 && error > 0) || (controllerOutputB < 0 && error < 0)){
            integratorSaturate = true;
        } else{
            integratorSaturate = false;
        }

        if(isSaturating && integratorSaturate){
            controllerOutput = Kp * error + Kd * derivative;
        }
        return controllerOutput;
    }
}
