package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.FreightFrenzyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.PurePursuit;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.PathBuilder;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.AutoTask;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.vision.aruco.ArucoPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AutoPaths extends AutoPipeline {

    PurePursuit follower = new PurePursuit(this);
    FreightFrenzyConfig subsystems = new FreightFrenzyConfig();
    OpenCvWebcam webcam;
    ArucoPipeline pipeline;
    ElapsedTime timer = new ElapsedTime();

    int autoCase = 3;

    public void runOpMode() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        robotInit();
        pipeline = new ArucoPipeline(telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(pipeline);


        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        waitForStart();

        int liftPos = 750;
        double linkagePos = 1.0;
        Pose2d capPos = new Pose2d();

        double markerPos = pipeline.markerPos.y;
        if(markerPos > 0 && markerPos < 150) {
            autoCase = 3;
        } else if(markerPos >= 150 && markerPos < 250) {
            autoCase = 2;
        } else {
            autoCase = 1;
        }
        telemetry.addData("auto case", autoCase);
        telemetry.update();

        switch (autoCase) {
            case 1:
                liftPos = 0;
                break;
            case 2:
                liftPos = 325;
                capPos = new Pose2d(-12.0, -1.5, 0.2);
                break;
            case 3:
                liftPos = 750;
                capPos = new Pose2d(-12.0, 6.0, 0.0);
                break;
        }

        int finalLiftPos = liftPos;
        subsystems.cap.setArmPosition(0.2);

        Path toCap = new PathBuilder()
                .speed(0.2)
                .maintainHeading(true)
                .start(new Pose2d(0.0, 0.0, 0.0))
                .addGuidePoint(new Pose2d(0.0, 0.0, 0.0))
                .end(capPos)
                .build();

        follower.followPath(toCap);

        timer.reset();
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() < 500;
            }

            @Override
            public void run() {
                double pos = 0.2 + (timer.milliseconds() / 1000);
                subsystems.cap.setArmPosition(pos);
            }
        });

        subsystems.lift.liftToPosition(liftPos);
        timer.reset();


        Path start = new PathBuilder()
                .speed(0.2)
                .turnSpeed(0.5)
                .lookahead(5)
                .maintainHeading(true)
                .start(capPos)
                .addGuidePoint(capPos)
                .end(new Pose2d(-20.0, 3.0, 5.74))
                .build();
        follower.followPath(start);
        timer.reset();
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() < 200;
            }

            @Override
            public void run() {
                subsystems.lift.setLinkagePosition(1.0);
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
                subsystems.intake.run(false, false, false);
                subsystems.lift.setReleasePosition(0.45);
            }
        });
        subsystems.lift.setLinkagePosition(0.49);

//        Path toDepot = toDepot();
//        follower.followPath(new Path(toDepot));
//
//        Path deposit = toDeposit();
//        follower.followPath(new Path(deposit));

        int i = 0;
        while(i < 3) {
            double depotPosX;
            switch (i) {
                case 1:
                    depotPosX = -7.0;
                    break;
                case 2:
                    depotPosX = -14;
                    break;
                default:
                    depotPosX = 0.9;
                    break;

            }
            Path toDepot = toDepot(depotPosX);
            toDepot.setPrecise(false);
            Path deposit = toDeposit(depotPosX);
            subsystems.lift.setReleasePosition(0.01);
           follower.followPath(new Path(toDepot));
            runTask(new AutoTask() {
               @Override
               public boolean conditional() {
                   return !subsystems.intake.indexerOn();
               }

               @Override
               public void run() {
                   double speed = 0.2;
                   double turn = 0.0;
                   subsystems.intake.run(true, false, false);
                   subsystems.drivetrain.setPowers(speed + turn, speed - turn, speed + turn, speed - turn);
               }
           });
           subsystems.intake.run(true, false, false);
           subsystems.intake.setIntakePower(-0.4);
           subsystems.intake.setTransferPower(1.0);
           follower.followPath(new Path(deposit));
           timer.reset();
           runTask(new AutoTask() {
               @Override
               public boolean conditional() {
                   return timer.milliseconds() < 400;
               }

               @Override
               public void run() {
                   subsystems.intake.run(false, false, false);
                   subsystems.lift.setReleasePosition(0.45);
               }
           });
            subsystems.lift.liftToPosition(800);
            subsystems.lift.setLinkagePosition(0.49);
            i++;
        }
        subsystems.lift.setReleasePosition(0.01);
        follower.followPath(toDepot());


//        runTask(new AutoTask() {
//            @Override
//            public boolean conditional() {
//                return !subsystems.intake.indexerOn();
//            }
//            @Override
//            public void run() {
//                subsystems.intake.run(true, false, false);
//            }
//        });
//        while(!isStopRequested()) {}

//        runTask(new AutoTask() {
//            @Override
//            public boolean conditional() {
//                return true;
//            }
//            @Override
//            public void run() {
//            }
//        });

        follower.following = false;
        Robot.drivetrain.setPowers(new double[] {0, 0, 0, 0});

        while(opModeIsActive()) {
            Robot.update();
            Pose2d robotPose = Robot.getRobotPose();
            telemetry.addData("robot x", robotPose.getX());
            telemetry.addData("robot y", robotPose.getY());
            telemetry.addData("robot theta", robotPose.getHeading());
            telemetry.update();

        }

    }

    private Path toDepot() {
        return toDepot(0.9);
    }

    private Path toDepot(double depotPosX) {
        return new PathBuilder()
                .speed(0.3)
                .turnSpeed(0.5)
                .lookahead(5.0)
                .maintainHeading(true)
                .start(new Pose2d(-21.0, 4.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .addGuidePoint(new Pose2d(-9.0, -0.4, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .speed(0.6)
                .addGuidePoint(new Pose2d(0.2, -2.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .speed(0.8)
                .addTask(() -> {
                    subsystems.intake.run(true, false, false);
                    subsystems.lift.liftToPosition(0);
                })
                .addGuidePoint(new Pose2d(0.51, -20.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .speed(0.4)
                .addGuidePoint(new Pose2d(0.5, -30.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .end(new Pose2d(depotPosX, -40.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .build();
    }

    private Path toDeposit() {
        return  toDeposit(0.9);
    }

    private Path toDeposit(double depotPosX) {
        return new PathBuilder()
                .speed(0.3)
                .turnSpeed(0.5)
                .maintainHeading(true)
                .start(new Pose2d(depotPosX, -40.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .addGuidePoint(new Pose2d(0.0, -35.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .speed(0.8)
                .addGuidePoint(new Pose2d(0.5, -35.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .addGuidePoint(new Pose2d(0.51, -20.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .speed(0.5)
                .addGuidePoint(new Pose2d(0.5, -8.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .addGuidePoint(new Pose2d(-9.6, -2.0, 5.69))
                .addTask(() -> {
                    subsystems.lift.liftToPosition(750);
                    subsystems.lift.setLinkagePosition(1.0);
                })
                .speed(0.3)
                .end(new Pose2d(-20.0, 4.0, 5.69))
                .build();
    }

}
