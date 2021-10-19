package org.firstinspires.ftc.teamcode.robotcorelib.motion;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robotcorelib.drive.DriveMode;

/**
 * Kinematics class that translates velocity vector to wheel velocities for different drivebases.
 * the order in which wheel velocities are returned is defined in DrivetrainInterface.
 * @see org.firstinspires.ftc.teamcode.robotcorelib.drive.DrivetrainInterface
 */
public class DriveKinematics {

    public static double[] tankVelocityToWheelVelocities(double forward, double turn) {
        double magnitude = Math.abs(forward) + Math.abs(turn);
        if(magnitude > 1.0) {
            forward *= 1.0 / magnitude;
            turn *= 1.0 / magnitude;
        }
        return new double[] {
                forward + turn,
                forward - turn,
                forward + turn,
                forward - turn
        };
    }

    public static double[] mecanumVelocityToWheelVelocities(Pose2d robotVelocity) {
        double multiplier = 2.0 / Math.sqrt(2.0);

    }

    public static double[] mecanumFieldVelocityToWheelVelocities(Pose2d robotFieldPose, Pose2d robotFieldVelocity) {

    }

}
