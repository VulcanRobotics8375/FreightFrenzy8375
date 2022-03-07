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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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
            .addGuidePoint(new Pose2d(0.0, 0.0, 0.0))
            .addTask(() -> {
                subsystems.lift.runTurretAndArm(false, true, false, 0.0, 0.0, false, false, false, false, true);
            })
            .end(new Pose2d(16.0, 8.0, 0))
            .build();
    Path allianceToWarehouse = new PathBuilder()
            .speed(1.0)
            .turnSpeed(0.5)
            .maintainHeading(true)
            .start(new Pose2d(16.0, 8.0, 0))
            .addGuidePoint(new Pose2d(16.0, 8.0, 0))
            .addTask(() -> {
                subsystems.intake.run(false, true, false);
                subsystems.lift.runTurretAndArm(false, false, true, 0.0, 0.0, false, true, false, false, false);
            })
            .addGuidePoint(new Pose2d(0,0.5,0))
            .addTask(() -> {
                subsystems.lift.runTurretAndArm(false, false, true, 0.0, 0.0, false, true, false, false, false);
            })
            .end(new Pose2d(-20,0.51,0))
            .build();
    Path warehouseToAlliance = new PathBuilder()
            .speed(1.0)
            .turnSpeed(0.5)
            .maintainHeading(true)
            .start(new Pose2d(-20,0.51,0))
            .addGuidePoint(new Pose2d(-20,0.51,0))
            .addTask(() -> {
                subsystems.lift.runTurretAndArm(false, false, false, 0.0, 0.0, false, false, false, false, false);
                subsystems.intake.run(false, false, subsystems.lift.isReset());
            })
            .addGuidePoint(new Pose2d(0,0.5,0))
            .addTask(() -> {
                subsystems.intake.run(false, false, false);
                subsystems.lift.runTurretAndArm(false, true, false, 0.0, 0.0, false, false, false, false, false);
            })
            .end(new Pose2d(16.0,12.0,0))
            .build();

    public void runOpMode() {
        allianceToWarehouse.setPrecise(false);
        warehouseToAlliance.setPrecise(false);

        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        robotInit();

        waitForStart();

        subsystems.intake.run(false, true, false);
        follower.followPath(startToAlliance);
        int cycle = 0;
        while(!isStopRequested()) {


            subsystems.lift.runTurretAndArm(false, true, false, 0.0, 0.0, false, false, false, false, false);

            runTask(new AutoTask() {
                @Override
                public boolean conditional() {
                    return Math.abs(subsystems.lift.getLiftPosition() - subsystems.lift.getLiftAlliancePos()) > 10
                            && Math.abs(subsystems.lift.getTurretPosition() - subsystems.lift.getTurret90Degrees()) > 10;
                }

                @Override
                public void run() {
                    subsystems.lift.runTurretAndArm(false, true, false, 0.0, 0.0, false, false, false, false, false);
                }
            });

                subsystems.lift.runTurretAndArm(false, true, false, 0.0, 0.0, true, false, false, false, false);
                timer.reset();
            runTask(new AutoTask() {
                @Override
                public boolean conditional() {
                    return timer.milliseconds() <= 500;
                }

                @Override
                public void run() {
                    subsystems.lift.runTurretAndArm(false, true, false, 0.0, 0.0, true, false, false, false, false);
                }
            });
            subsystems.lift.runTurretAndArm(false, false, false, 0.0, 0.0, false, true, false, false, false);

            timer.reset();
            runTask(new AutoTask() {
                @Override
                public boolean conditional() {
                    return timer.milliseconds() < 200;
                }

                @Override
                public void run() {

                }
            });
            timer.reset();

            runTask(new AutoTask() {
                @Override
                public boolean conditional() {
                    return timer.milliseconds() < 500;
                }

                @Override
                public void run() {
                    subsystems.lift.runTurretAndArm(false, true, false, 0.0, 0.0, true, false, false, false, false);
                }
            });

            subsystems.lift.runTurretAndArm(false, false, true, 0.0, 0.0, false, false, false, false, false);
            follower.followPath(allianceToWarehouse);

            subsystems.intake.run(true, false, false);
            double turn = 0.1 * cycle;
            runTask(new AutoTask() {
                @Override
                public boolean conditional() {
                    return subsystems.intake.getIntakeState() != Intake.IntakeState.INDEXED;
                }

                @Override
                public void run() {
                    subsystems.drivetrain.setPowers(0.25 + turn, 0.25, 0.25 + turn, 0.25);
                    subsystems.intake.run(false, false, false);
                }
            });
            subsystems.intake.run(false, false, false);
                Pose2d robotPose = Robot.getRobotPose();
                double robotAngle = Math.toRadians(subsystems.drivetrain.getIMU().getAngularOrientation().firstAngle);
                Robot.setRobotPose(new Pose2d(robotPose.getX(), robotPose.getY() - 1.0, robotAngle));

    //
                follower.followPath(warehouseToAlliance);
                cycle++;
        }

        telemetry.addLine("done");
        subsystems.drivetrain.setPowers(0, 0, 0, 0);
        Robot.stop();
    }

}
