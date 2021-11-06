package org.firstinspires.ftc.teamcode.robotcorelib.math;

import java.util.*;
import java.io.*;

public class PID {
    private double Kp, Ki, Kd;
    private double outputMaxLimit, outputMinLimit;
    private double error, integralError, lastError;
    private double integral, derivative;
    private double controllerOutput, controllerOutputB;
    private boolean isSaturating = false;
    private boolean integratorSaturate = false;
    private boolean integralClamp = false;

    public PID(double Kp, double Ki, double Kd, double outputMaxLimit, double outputMinLimit){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.outputMaxLimit = outputMaxLimit;
        this.outputMinLimit = outputMinLimit;
    }

    public double run(double target, double measurement){
        error = target - measurement;
        if(!integralClamp){
            integralError = (error + lastError) / 2.0;
        }else{
            integralError = 0;
        }
        integral += integralError;
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

        if(Ki * integral > Kp * error){
            integral = error * (Kp/Ki);
            controllerOutput = Kp * error + Ki * integral + Kd * derivative;
        }

        if((controllerOutputB > 0 && error > 0) || (controllerOutputB < 0 && error < 0)){
            integratorSaturate = true;
        } else{
            integratorSaturate = false;
        }

        if(isSaturating && integratorSaturate){
            integralClamp = true;
        }else{
            integralClamp = false;
        }
        return controllerOutput;
    }

    public double getIntegral(){
        return integral;
    }
    public double getIntegralError(){
        return integralError;
    }
}
