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
import org.firstinspires.ftc.teamcode.vision.aruco.ArucoPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Red - Duck Side", group = "red")
public class FlippedDuckAuto extends AutoPipeline {
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
            .end(new Pose2d(-6.0, 15.0, 0))
            .build();

    Path allianceToDuck = new PathBuilder()
            .speed(0.6)
            .turnSpeed(0.5)
            .maintainHeading(true)
            .start(new Pose2d(-6.0, 15.0, 0))
            .addGuidePoint(new Pose2d(-18.0, 8.0, 0))
            .addTask(() -> {
                subsystems.lift.runTurretAndArm();
            })
            .end(new Pose2d(-22, 6, 0))
            .build();

    Path duckToPark = new PathBuilder()
            .speed(0.5)
            .turnSpeed(0.5)
            .maintainHeading(true)
            .start(new Pose2d(-22.0, 6.0, 0))
            .addGuidePoint(new Pose2d(-22.01, 6.01, 0))
            .end(new Pose2d(-28.0, 29.0, 0))
            .build();

    public void runOpMode() {
        msStuckDetectStop = 2000;
        int preloadHeight;
//        double preloadHeight = 220;
//        double preloadHeight = subsystems.lift.getLiftAlliancePos();
        allianceToDuck.setPrecise(false);
        duckToPark.setPrecise(false);

        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        robotInit();
        subsystems.lift.autoMode();
        int turretAlliancePos = -1 * (subsystems.lift.getTurret90Degrees() + (int) subsystems.lift.getTurretOffset() - 200);

        pipeline.boundingBoxBoundaryOne = 78;
        pipeline.boundingBoxBoundaryTwo = 175;

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
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        subsystems.intake.run(false, true, false);
        waitForStart();

        double markerPos = 0;
        if (pipeline.markerPos != null) {
            markerPos = pipeline.markerPos.x;
        }

        if (markerPos < pipeline.boundingBoxBoundaryOne) {
            preloadHeight = 70;
        } else if (markerPos >= pipeline.boundingBoxBoundaryOne && markerPos < pipeline.boundingBoxBoundaryTwo) {
            preloadHeight = 220;
        } else {
            preloadHeight = subsystems.lift.getLiftAlliancePos();
        }

        // Go to preload and start bringing out lift
        subsystems.intake.run(false, true, false);
        subsystems.lift.liftToPosition(subsystems.lift.getLiftSharedPos());

        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return Math.abs(subsystems.lift.getLiftPosition() - subsystems.lift.getLiftSharedPos()) >= 10;
            }

            @Override
            public void run() {
            }
        });

        subsystems.lift.turretToPosition(turretAlliancePos);
        follower.followPath(startToAlliance);

        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return Math.abs(subsystems.lift.getTurretPosition() - turretAlliancePos) >= 20;
            }

            @Override
            public void run() {
            }
        });

        subsystems.lift.liftToPosition(preloadHeight, 1.0);
        subsystems.lift.setLinkagePos(0.9);

        timer.reset();
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() <= 600;
            }

            @Override
            public void run() {
            }
        });

        subsystems.lift.setReleasePosition(0.27);

        timer.reset();
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() <= 600;
            }

            @Override
            public void run() {
            }
        });

        subsystems.lift.liftToPosition(subsystems.lift.getLiftSharedPos(), 1.0);
        subsystems.lift.setReleasePosition(0.4);
        subsystems.lift.setLinkagePos(0.1);

        timer.reset();
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() <= 2000;
            }

            @Override
            public void run() {
            }
        });

        subsystems.lift.runTurretAndArm(false, false, true, 0.0, 0.0, false, false, 0.0, 0.0, false);
        follower.followPath(allianceToDuck);

        subsystems.carousel.setCarouselPower(1);
        timer.reset();
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return timer.milliseconds() <= 6000;
            }

            @Override
            public void run() {

            }
        });
        subsystems.carousel.setCarouselPower(0);

        follower.followPath(duckToPark);

        subsystems.lift.runTurretAndArm(false, false, true, 0.0, 0.0, false, false, 0.0, 0.0, false);
        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return !subsystems.lift.isReset();
            }

            @Override
            public void run() {
                subsystems.lift.runTurretAndArm();
            }
        });

        subsystems.drivetrain.setPowers(0,0,0,0);

        telemetry.addLine("done");
        Robot.stop();

    }

}
