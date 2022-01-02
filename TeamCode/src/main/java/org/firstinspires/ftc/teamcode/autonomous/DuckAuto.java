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
import org.firstinspires.ftc.teamcode.robotcorelib.util.AutoTask;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;
import org.firstinspires.ftc.teamcode.vision.aruco.ArucoPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Blue - Depot Side", group = "blue")
public class DuckAuto extends AutoPipeline {

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
        if (markerPos > 0 && markerPos < 150) {
            autoCase = 3;
        } else if (markerPos >= 150 && markerPos < 250) {
            autoCase = 2;
        } else {
            autoCase = 1;
        }
        telemetry.addData("auto case", autoCase);
        telemetry.update();

        switch (autoCase) {
            case 1:
                liftPos = 0;
                capPos = new Pose2d(-13.5, -7.0, 0.4);
                linkagePos = 0.85;
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
                .speed(0.4)
                .turnSpeed(0.5)
                .lookahead(5)
                .maintainHeading(true)
                .start(capPos)
                .addGuidePoint(capPos)
                .end(new Pose2d(-20.0, -21.0, 0.72))
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

        Path toCarousel = new PathBuilder()
                .speed(0.5)
                .turnSpeed(0.5)
                .maintainHeading(true)
                .start(new Pose2d(-23.0, -16.0, 0.72))
                .addGuidePoint(new Pose2d(-23.0, -16.0, 0.72))
                .end(new Pose2d(-8.0, 8.0, Math.PI / 2.0))
                .build();

        follower.followPath(new Path(toCarousel));

        subsystems.lift.setReleasePosition(0.01);
        subsystems.lift.liftToPosition(0);
        subsystems.carousel.setOpenerPosition(0.2);
        subsystems.carousel.setCarouselPower(-1.0);

        timer.reset();
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() < 5000;
            }

            @Override
            public void run() {

            }
        });
        subsystems.carousel.setCarouselPower(0.0);
        subsystems.carousel.setOpenerPosition(1.0);

        Path intakeDuckPhaseOne = new PathBuilder()
                .speed(0.5)
                .turnSpeed(0.5)
                .maintainHeading(true)
                .start(new Pose2d(-8.0, 8.0, Math.PI / 2.0))
                .addGuidePoint(new Pose2d(-12.0, -4.0, Math.PI / 2.0))
                .end(new Pose2d(-3.0, -12.0, Math.PI / 2.0))
                .build();

        subsystems.intake.run(true, false, false);
        follower.followPath(intakeDuckPhaseOne);

        Path intakeDuckPhaseTwo = new PathBuilder()
                .speed(0.3)
                .turnSpeed(0.5)
                .maintainHeading(true)
                .start(new Pose2d(-3.0, -12.0, 0.9))
                .addGuidePoint(new Pose2d(-3.0, -12.0, 0.9))
                .end(new Pose2d(-2.0, 8.0, 0.9))
                .build();

        follower.followPath(intakeDuckPhaseTwo);

        Path fromIntakeToDeposit = new PathBuilder()
                .speed(0.4)
                .turnSpeed(0.5)
                .maintainHeading(true)
                .start(new Pose2d(-3.0, 8.0, 1.0))
                .addGuidePoint(new Pose2d(-3.0, 8.0, 0.72))
                .end(new Pose2d(-20.0, -21.0, 0.72))
                .build();

        follower.followPath(fromIntakeToDeposit);

        timer.reset();

        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() < 1000;
            }

            @Override
            public void run() {
                subsystems.lift.liftToPosition(750);
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

        subsystems.lift.liftToPosition(0);

        Path park = new PathBuilder()
                .speed(0.3)
                .turnSpeed(0.5)
                .maintainHeading(true)
                .start(new Pose2d(-20.0, 21.0, Math.PI / 2.0))
                .addGuidePoint(new Pose2d(-20.0, 21.0, Math.PI / 2.0))
                .end(new Pose2d(-31.0, 15.0, Math.PI / 2.0))
                .build();

        follower.followPath(new Path(park));

        while(opModeIsActive()) {

        }

    }
}
