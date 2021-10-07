package org.firstinspires.ftc.teamcode.robotcorelib.util.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LVMaxbotixEZ4 extends AnalogInput{


    public static final double PRE_SCALAR_INCHES = 512.0;
    public DistanceUnit distanceUnit = DistanceUnit.INCH;

    /**
     * Constructor
     *
     * @param controller AnalogInput controller this channel is attached to
     * @param channel    channel on the analog input controller
     */
    public LVMaxbotixEZ4(AnalogInputController controller, int channel) {
        super(controller, channel);
    }

    public double getDistance(DistanceUnit distanceUnit) {
        double defaultOut = getVoltage() * (PRE_SCALAR_INCHES / getMaxVoltage());
        switch(distanceUnit) {
            case CM:
                return defaultOut * 2.54;
            case MM:
                return defaultOut * 25.4;
            case METER:
                return defaultOut * 0.254;
            default:
                return defaultOut;
        }
    }

    public double getDistance() {
        return getDistance(distanceUnit);
    }

}
