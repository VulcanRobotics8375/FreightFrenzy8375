package org.firstinspires.ftc.teamcode.robotcorelib.drive;

import com.qualcomm.robotcore.hardware.DcMotor;

public interface DrivetrainInterface {

    //General Convention for writing setPowers() method:
    //4 motors(most drivetrains)-- front left, front right, back left, back right
    //if you have a diff configuration, add your new drive mode to DriveMode, and add update the kinematics for the path followers.
    //TODO add kinematics class for diff drivemodes?

    void setPowers(double[] powers);

    DriveMode getDriveMode();

    //might not need more than this honestly. Really depends on what we need tho.

}
