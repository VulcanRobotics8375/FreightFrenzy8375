package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.FreightFrenzyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.PurePursuit;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.PathBuilder;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.AutoTask;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.vision.aruco.ArucoPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Blue - Warehouse Side", group = "blue")
public class CabbageAuto extends AutoPipeline {
    PurePursuit follower = new PurePursuit(this);
    FreightFrenzyConfig subsystems = new FreightFrenzyConfig();
    OpenCvWebcam webcam;
    ArucoPipeline pipeline;
    ElapsedTime timer = new ElapsedTime();

    Path startToAlliance = new PathBuilder()
            .speed(0.8)
            .turnSpeed(0.5)
            .maintainHeading(true)
            .start(new Pose2d(0.0, 0.0, 0.0))
            .addGuidePoint(new Pose2d(10.0, 0.01, 0.0))
            .end(new Pose2d(24.01, 12.0, 0))
            .build();
    Path allianceToWarehouse = new PathBuilder()
            .speed(0.8)
            .turnSpeed(0.5)
            .maintainHeading(true)
            .start(new Pose2d(24.0, 12.0, 0))
            .addGuidePoint(new Pose2d(0,0.5,0))
            .end(new Pose2d(-30,0.51,0))
            .build();
    Path warehouseToAlliance = new PathBuilder()
            .speed(0.8)
            .turnSpeed(0.5)
            .maintainHeading(true)
            .start(new Pose2d(-30,0.51,0))
            .addGuidePoint(new Pose2d(0,0.5,0))
            .end(new Pose2d(24,12,0))
            .build();

    public void runOpMode() {

        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        robotInit();

        waitForStart();

        while(!isStopRequested()) {
            subsystems.intake.run(false, true, false);
            follower.followPath(startToAlliance);

//        runTask(new AutoTask() {
//            @Override
//            public boolean conditional() {
//                return subsystems.lift;
//            }
//
//            @Override
//            public void run() {
//                subsystems.lift.runTurretAndArm(false, true, false, 0.0, 0.0, false, false, false, false, false);
//                subsystems.lift.
//            }
//        });

            follower.followPath(allianceToWarehouse);
            Pose2d robotPose = Robot.getRobotPose();
            Robot.setRobotPose(new Pose2d(robotPose.getX(), -0.5, robotPose.getHeading()));
//
            follower.followPath(warehouseToAlliance);
        }

        telemetry.addLine("done");
        subsystems.drivetrain.setPowers(0, 0, 0, 0);
        Robot.stop();
    }

}
