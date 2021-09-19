package org.firstinspires.ftc.teamcode.robotcorelib.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;

import java.util.ArrayList;

public abstract class Subsystem {

    protected HardwareMap hardwareMap;

    protected Telemetry telemetry;

    public Subsystem() {
        Robot.getConfiguration().subsystems.add(this);
    }

    public abstract void init();

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

}
