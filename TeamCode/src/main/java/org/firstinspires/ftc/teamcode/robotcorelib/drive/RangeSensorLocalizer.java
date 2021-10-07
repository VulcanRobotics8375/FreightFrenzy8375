package org.firstinspires.ftc.teamcode.robotcorelib.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.LVMaxbotixEZ4;

import java.util.Arrays;
import java.util.List;

/**
 * Range Sensor localization using 2 walls.
 *
 * Mode Enum Reference:
 * ----------------------
 * |        north       |
 * |          |         |
 * |          |         |
 * |   west -- -- east  |
 * |          |         |
 * |          |         |
 * |        south       |
 * ----------------------
 *        AUDIENCE
 *
 * for freight frenzy, the north-west corner would be the blue alliance warehouse.
 * the order (NORTH_WEST vs WEST_NORTH) refers to what set of sensors is facing towards the wall.
 * The sensors facing forward (or backward) on the wall come first, and the perpendicular(normal)
 * sensors are the second direction. ( FORWARD_NORMAL )
 *
 */
public class RangeSensorLocalizer {
    public static double LATERAL_DISTANCE_FRONT = 1;
    public static double LATERAL_DISTANCE_SIDE = 1;
    public static double ANGLE_THRESHOLD = Math.toRadians(15);

    public DistanceUnit distanceUnit = DistanceUnit.CM;
    private Pose2d poseEstimate = new Pose2d();
    public String configurationValidator;

    private LVMaxbotixEZ4 forward_1, forward_2, normal_1, normal_2;


    enum Mode {
        NORTH_WEST, //forward sensors on north wall, normal sensors on west wall
        NORTH_EAST,
        SOUTH_WEST,
        SOUTH_EAST,
        EAST_NORTH,
        EAST_SOUTH,
        WEST_NORTH,
        WEST_SOUTH
    }

    static class ConfigurationValidator {
        public boolean pass;
        public String message;

        public ConfigurationValidator(boolean pass, String message) {
            this.pass = pass;
            this.message = message;
        }

        @Override
        public String toString() {
            if(pass) {
                return "Configuration Valid";
            } else {
                return "Localizer Configuration Error: " + message;
            }
        }

    }

    public RangeSensorLocalizer(HardwareMap hardwareMap) {
        // sensor positions relative to the center of the robot.
        // This is different from dead-wheel odo because it is not the center of rotation.
        List<Pose2d> sensorPoses = Arrays.asList(
                new Pose2d(0, 0, 0),
                new Pose2d(0, 0, 0),
                new Pose2d(0, 0, Math.toRadians(90)),
                new Pose2d(0, 0, Math.toRadians(90))
        );
        configurationValidator = validateConfiguration(sensorPoses).toString();

        forward_1 = (LVMaxbotixEZ4) hardwareMap.analogInput.get("forward_1");
        forward_2 = (LVMaxbotixEZ4) hardwareMap.analogInput.get("forward_2");
        normal_1 = (LVMaxbotixEZ4) hardwareMap.analogInput.get("normal_1");
        normal_2 = (LVMaxbotixEZ4) hardwareMap.analogInput.get("normal_2");

    }

    private Pose2d estimatePose() {
        List<Double> distances = getDistances();


    }

    private ConfigurationValidator validateConfiguration(List<Pose2d> sensorPositions) {
        if(sensorPositions.size() != 4) {
            return new ConfigurationValidator(false, "localizer requires 4 sensors");
        }
        else {
            int forwardNum = 0;
            int normalNum = 0;
            for (int i = 0; i < 4; i++) {
                Vector2d headingVec = sensorPositions.get(i).headingVec();
                if(Math.abs(headingVec.getY()) < Math.sin(ANGLE_THRESHOLD)) {
                    forwardNum++;
                } else if(Math.abs(headingVec.getX()) < Math.cos(ANGLE_THRESHOLD)) {
                    normalNum++;
                }
            }
            if(forwardNum != 2 || normalNum != 2) {
                return new ConfigurationValidator(false, "Cannot solve localization with this configuration");
            }
        }
        return new ConfigurationValidator(true, "config passed");
    }

    public List<Double> getDistances() {
        return Arrays.asList(
                forward_1.getDistance(distanceUnit),
                forward_2.getDistance(distanceUnit),
                normal_1.getDistance(distanceUnit),
                normal_2.getDistance(distanceUnit)
        );
    }

    public void setDistanceUnit(DistanceUnit unit) {
        this.distanceUnit = unit;
    }

}
