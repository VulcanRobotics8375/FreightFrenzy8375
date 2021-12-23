package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.FreightFrenzyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.math.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.PurePursuit;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.kinematics.DriveKinematics;
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

@Autonomous(name = "Blue - Warehouse Side", group = "blue")
public class NewBadBitchAuto extends AutoPipeline {

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

//        while(subsystems.drivetrain.getIMU().isGyroCalibrated()) {}

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
                capPos = new Pose2d(-13.5, -5.0, 0.4);
                linkagePos = 0.9;
                break;
            case 2:
                liftPos = 325;
                capPos = new Pose2d(-12.0, 0.5, 0.2);
                break;
            case 3:
                liftPos = 750;
                capPos = new Pose2d(-12.0, 6.0, 0.0);
                break;
        }

        subsystems.cap.setArmPosition(0.2);
        Path toCap = new PathBuilder()
                .speed(0.35)
                .turnSpeed(0.5)
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
                double pos = 0.2 + (timer.milliseconds() / 1000.0);
                subsystems.cap.setArmPosition(pos);
            }
        });

        subsystems.lift.liftToPosition(liftPos);
        timer.reset();

        Path start = new PathBuilder()
                .speed(0.5)
                .turnSpeed(0.5)
                .lookahead(5)
                .maintainHeading(true)
                .start(capPos)
                .addGuidePoint(capPos)
                .end(new Pose2d(-22.0, 4.0, 5.74))
                .build();
        follower.followPath(start);
        timer.reset();
        double finalLinkagePos = linkagePos;
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() < 200;
            }

            @Override
            public void run() {
                subsystems.lift.setLinkagePosition(finalLinkagePos);
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
                subsystems.lift.setReleasePosition(0.45);
            }
        });
        timer.reset();
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() < 200;
            }

            @Override
            public void run() {
                subsystems.lift.setLinkagePosition(0.49);
            }
        });


        //START OF LOOP
        int i = 0;
        while(i < 3) {
            subsystems.lift.setReleasePosition(0.0);
            //WAREHOUSE CYCLE 1
            timer.reset();
            Path toWarehouse = new PathBuilder()
                    .speed(0.6)
                    .turnSpeed(0.5)
                    .maintainHeading(true)
                    .start(new Pose2d(-22.0, 4.0, 5.74))
                    .addGuidePoint(new Pose2d(1.0, -6.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                    .addTask(() -> {
                        subsystems.lift.liftToPosition(0);
                        subsystems.intake.run(true, false, false);
                    })
                    .addGuidePoint(new Pose2d(1.01, -30, (2.0 * Math.PI) - (Math.PI / 2.0)))
                    .speed(0.35)
                    .end(new Pose2d(-1.0, -45.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                    .build();
            toWarehouse.setPrecise(false);
            follower.followPath(toWarehouse);

            //INTAKE SUBROUTINE
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
    //                       subsystems.drivetrain.setPowers(-(speed + turn), -(speed - turn), -(speed + turn), -(speed - turn));
                }
            });
            subsystems.intake.run(true, false, false);
            subsystems.intake.setIntakePower(-0.4);
            subsystems.intake.setTransferPower(1.0);

            //DEPOSIT CYCLE 1
            Path toDeposit = new PathBuilder()
                    .speed(0.6)
                    .turnSpeed(0.5)
                    .maintainHeading(true)
                    .start(new Pose2d(1.0, -45.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                    .speed(0.5)
                    .addGuidePoint(new Pose2d(1.01, -7.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                    .addTask(() -> {
                        subsystems.lift.liftToPosition(750);
                        subsystems.lift.setLinkagePosition(1.0);
                    })
//                    .addGuidePoint(new Pose2d(-12.0, -3.0, 5.74))
//                    .speed(0.35)
                    .end(new Pose2d(-25.0, 1.0, 5.74))
                    .build();
            follower.followPath(toDeposit);

            //DEPOSIT SUBROUTINE
            timer.reset();
            runTask(new AutoTask() {
                @Override
                public boolean conditional() {
                    return timer.milliseconds() < 250;
                }

                @Override
                public void run() {
                    subsystems.intake.run(false, false, false);
                    subsystems.lift.setReleasePosition(0.45);
                }
            });
//        subsystems.lift.liftToPosition(800);
            subsystems.lift.setLinkagePosition(0.49);
            subsystems.lift.liftToPosition(800);
            i++;
        }
        subsystems.lift.setReleasePosition(0.0);
        //WAREHOUSE CYCLE 1
        timer.reset();
        Path toWarehouse = new PathBuilder()
                .speed(0.6)
                .turnSpeed(0.5)
                .maintainHeading(true)
                .start(new Pose2d(-22.0, 4.0, 5.74))
                .addGuidePoint(new Pose2d(1.0, -6.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .addTask(() -> {
                    subsystems.lift.liftToPosition(0);
//                    subsystems.intake.run(true, false, false);
                })
                .addGuidePoint(new Pose2d(1.01, -30, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .speed(0.35)
                .end(new Pose2d(1.0, -45.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .build();
        toWarehouse.setPrecise(false);
        follower.followPath(toWarehouse);
    }

}
