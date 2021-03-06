package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Blue - Warehouse Side", group = "blue")
public class CabbageAuto extends AutoPipeline {
    PurePursuit follower = new PurePursuit(this);
    FreightFrenzyConfig subsystems = new FreightFrenzyConfig();
    OpenCvWebcam webcam;
    ArucoPipeline pipeline = new ArucoPipeline(telemetry);
    ElapsedTime timer = new ElapsedTime();

    Path startToAlliance = new PathBuilder()
            .speed(0.8)
            .turnSpeed(0.5)
            .maintainHeading(true)
            .start(new Pose2d(0.0, 0.0, 0.0))
            .addGuidePoint(new Pose2d(0.0, 0.0, 0.0))
            .addTask(() -> {
                subsystems.lift.runTurretAndArm();
            })
            .end(new Pose2d(15.0, 8.0, 0))
            .build();
    Path allianceToWarehouse = new PathBuilder()
            .speed(1.0)
            .turnSpeed(0.5)
            .maintainHeading(true)
            .start(new Pose2d(13.0, 8.0, 0))
            .addGuidePoint(new Pose2d(13.0, 8.0, 0))
            .addTask(() -> {
                subsystems.intake.run(false, true, false);
                subsystems.lift.runTurretAndArm();
            })
            .addGuidePoint(new Pose2d(0,0.5,0))
            .addTask(() -> { subsystems.lift.runTurretAndArm(); })
            .end(new Pose2d(-20,0.51,0))
            .build();
    Path warehouseToAlliance = new PathBuilder()
            .speed(1.0)
            .turnSpeed(0.5)
            .maintainHeading(true)
            .start(new Pose2d(-20,0.51,0))
            .addGuidePoint(new Pose2d(-20,0.51,0))
            .addTask(() -> {
                subsystems.lift.runTurretAndArm();
                subsystems.intake.run(false, false, subsystems.lift.isReset());
            })
            .addGuidePoint(new Pose2d(0,0.5,0))
            .addTask(() -> {
                subsystems.lift.runTurretAndArm(false, true, false, 0.0, 0.0, false, false, 0.0, 0.0, false);
                subsystems.intake.run(false, false, false);
            })
            .end(new Pose2d(13.5,13.0,0))
            .build();

    public void runOpMode() {
        msStuckDetectStop = 2000;
        int preloadHeight;
        double preloadLinkagePos;
//        double preloadHeight = 220;
//        double preloadHeight = subsystems.lift.getLiftAlliancePos();
        allianceToWarehouse.setPrecise(false);
        warehouseToAlliance.setPrecise(false);

        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        robotInit();
        subsystems.lift.autoMode();

        pipeline.boundingBoxBoundaryOne = 78;
        pipeline.boundingBoxBoundaryTwo = 175;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        waitForStart();

        double markerPos = pipeline.markerPos.x;

        if(markerPos < pipeline.boundingBoxBoundaryOne) {
            preloadHeight = 70;
            preloadLinkagePos = 0.78;
        } else if(markerPos >= pipeline.boundingBoxBoundaryOne && markerPos < pipeline.boundingBoxBoundaryTwo) {
            preloadHeight = 220;
            preloadLinkagePos = 0.83;
        } else {
            preloadHeight = subsystems.lift.getLiftAlliancePos();
            preloadLinkagePos = 0.9;
        }

        // Go to preload and start bringing out lift/
        subsystems.lift.runTurretAndArm(true, false, false, 0.0, 0.0, false, false, 0.0, 0.0, true);
        subsystems.intake.run(false, true, false);
        follower.followPath(startToAlliance);

        int cycle = 0;
        while(cycle <= 4 && !isStopRequested()) {
            // Bring lift and turret all the way out
            double targetLiftPos = cycle == 0 ? subsystems.lift.getLiftSharedPos() : subsystems.lift.getLiftAlliancePos();
            telemetry.addData("lift waiting", "true");
            runTask(new AutoTask() {
                @Override
                public boolean conditional() {
                    return Math.abs(subsystems.lift.getLiftPosition()) <= Math.abs(targetLiftPos) - 20;
                }
                @Override
                public void run() {
                    subsystems.lift.runTurretAndArm();
                    telemetry.addData("lift error", Math.abs(subsystems.lift.getLiftPosition()) - Math.abs(targetLiftPos));
                }
            });
            telemetry.addData("lift waiting", "false");

            if(cycle == 0) {
                subsystems.lift.liftToPosition(preloadHeight, 1.0);
                timer.reset();
                runTask(new AutoTask() {
                    @Override
                    public boolean conditional() {
                        return timer.milliseconds() <= 600;
                    }

                    @Override
                    public void run() {
                        subsystems.lift.setLinkagePos(preloadLinkagePos);
                    }
                });
                timer.reset();
                runTask(new AutoTask() {
                    @Override
                    public boolean conditional() {
                        return timer.milliseconds() <= 350;
                    }

                    @Override
                    public void run() {
                        subsystems.lift.setReleasePosition(0.27);
                    }
                });

            } else {
                // Bring linkage out
                timer.reset();
                runTask(new AutoTask() {
                    @Override
                    public boolean conditional() {
                        return timer.milliseconds() <= 350;
                    }

                    @Override
                    public void run() {
                        subsystems.lift.runTurretAndArm();
                    }
                });
                // Hopper OPEN
                subsystems.lift.runTurretAndArm(false, false, false, 0.0, 0.0, false, true, 0.0, 0.0, false);
                timer.reset();
                runTask(new AutoTask() {
                    @Override
                    public boolean conditional() {
                        return timer.milliseconds() <= 200;
                    }

                    @Override
                    public void run() {
                        subsystems.lift.runTurretAndArm();
                    }
                });
            }

            // Linkage + Hopper reset
            if(cycle == 0) {
                subsystems.lift.runTurretAndArm(false, false, false, 0.0, 0.0, false, true, 0.0, 0.0, false);
            } else {
                subsystems.lift.runTurretAndArm(false, false, false, 0.0, 0.0, true, true, 0.0, 0.0, false);
            }
            timer.reset();
            runTask(new AutoTask() {
                @Override
                public boolean conditional() {
                    return timer.milliseconds() <= 450;
                }
                @Override
                public void run() {
                    subsystems.lift.runTurretAndArm();
                }
            });

            // Lift + Turret reset, go from alliance to warehouse
            subsystems.lift.runTurretAndArm(false, false, true, 0.0, 0.0, false, false, 0.0, 0.0, false);
            follower.followPath(allianceToWarehouse);

            // Intake, moving forward slowly until indexed
            double turn = 0.08 * cycle;
            subsystems.intake.setExtendForwardPosition(subsystems.intake.getExtendForwardPosition() + 0.06);
            if(turn >= (0.08 * 3)) {
                turn = 0.0;
                subsystems.intake.setExtendForwardPosition(0.05);
            }

            double finalTurn = turn;
            runTask(new AutoTask() {
                @Override
                public boolean conditional() {
                    return subsystems.intake.getIntakeState() != Intake.IntakeState.INDEXED;
                }
                @Override
                public void run() {
                    subsystems.drivetrain.setPowers(0.25 + finalTurn, 0.25 - finalTurn, 0.25 + finalTurn, 0.25 - finalTurn);
                    subsystems.intake.run(true, false, false);
                }
            });

            // Use IMU to correct heading
            Pose2d robotPose = Robot.getRobotPose();
            double robotAngle = Math.toRadians(subsystems.drivetrain.getIMU().getAngularOrientation().firstAngle);
            Robot.setRobotPose(new Pose2d(robotPose.getX(), robotPose.getY() - 1.0, robotAngle));


            follower.followPath(warehouseToAlliance);
            cycle++;
        }
        subsystems.drivetrain.setPowers(0, 0, 0, 0);

        telemetry.addLine("done");
        Robot.stop();
    }

}
