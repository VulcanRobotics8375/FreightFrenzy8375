package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.FreightFrenzyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.PurePursuit;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.PathBuilder;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.AutoPipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.AutoTask;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@Autonomous
public class AutoPaths extends AutoPipeline {

    PurePursuit follower = new PurePursuit(this);
    FreightFrenzyConfig subsystems = new FreightFrenzyConfig();

    PathBuilder toDepot = new PathBuilder()
            .speed(0.4)
            .turnSpeed(0.5)
            .lookahead(5.0)
            .maintainHeading(true)
            .start(new Pose2d(-21.0, 4.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
            .addGuidePoint(new Pose2d(0.9, -2.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
            .addTask(() -> {
                subsystems.intake.run(true, false, false);
                subsystems.lift.setLinkagePosition(0.49);
            })
            .speed(0.8)
            .addGuidePoint(new Pose2d(0.8, -20.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
            .speed(0.2)
            .end(new Pose2d(0.9, -40.0, (2.0 * Math.PI) - (Math.PI / 2.0)));

    PathBuilder toDeposit = new PathBuilder()
            .speed(1.0)
            .turnSpeed(0.5)
            .lookahead(5.0)
            .maintainHeading(true)
            .start(new Pose2d(0.9, -40.0, 0))
            .addGuidePoint(new Pose2d(0.8, -30.0, 0))
            .speed(0.5)
            .addGuidePoint(new Pose2d(0.9, -8.0, 0))
            .addTask(() -> {
                subsystems.intake.run(false, false, false);
            })
            .speed(1.0)
            .end(new Pose2d(-21.0, 4.0, 0));

    public void runOpMode() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        robotInit();

        waitForStart();

        Path start = new PathBuilder()
                .speed(0.5)
                .turnSpeed(0.5)
                .lookahead(5)
                .maintainHeading(true)
                .lineToConstantHeading(
                        new Pose2d(0, 0, 5.67),
                        new Pose2d(-21.0, 4.0, 5.67))
                .build();
        follower.followPath(start);

//        Path toDepot = toDepot();
//        follower.followPath(new Path(toDepot));
//
//        Path deposit = toDeposit();
//        follower.followPath(new Path(deposit));

        int i = 0;
        while(i < 4) {
            double depotPosX;
            switch (i) {
                case 1:
                    depotPosX = -10.0;
                    break;
                case 2:
                    depotPosX = -20.0;
                    break;
                default:
                    depotPosX = 0.9;
                    break;

            }
            Path toDepot = toDepot(depotPosX);
            toDepot.setPrecise(false);
            Path deposit = toDeposit(depotPosX);
           follower.followPath(new Path(toDepot));
            subsystems.lift.setReleasePosition(0.01);
            runTask(new AutoTask() {
               @Override
               public boolean conditional() {
                   return !subsystems.intake.indexerOn();
               }

               @Override
               public void run() {
                   double speed = 0.3;
                   double turn = 0.0;
                   subsystems.intake.run(true, false, false);
                   subsystems.drivetrain.setPowers(speed + turn, speed - turn, speed + turn, speed - turn);
               }
           });
           subsystems.intake.run(true, false, false);
           subsystems.intake.setIntakePower(-0.4);
           follower.followPath(new Path(deposit));
            ElapsedTime timer = new ElapsedTime();
            timer.reset();
           runTask(new AutoTask() {
               @Override
               public boolean conditional() {
                   return timer.milliseconds() < 350;
               }

               @Override
               public void run() {
                   subsystems.lift.setReleasePosition(0.45);
               }
           });
            i++;
        }
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
        return toDepot.build();
    }

    private Path toDepot(double depotPosX) {
        return new PathBuilder()
                .speed(0.4)
                .turnSpeed(0.5)
                .lookahead(5.0)
                .maintainHeading(true)
                .start(new Pose2d(-21.0, 4.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .addGuidePoint(new Pose2d(0.9, -2.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .addTask(() -> {
                    subsystems.intake.run(true, false, false);
                    subsystems.lift.setLinkagePosition(0.49);
                })
                .speed(0.8)
                .addGuidePoint(new Pose2d(0.8, -20.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .speed(0.2)
                .end(new Pose2d(depotPosX, -40.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .build();
    }

    private Path toDeposit() {
        return  toDeposit.build();
    }

    private Path toDeposit(double depotPosX) {
        return new PathBuilder()
                .speed(0.3)
                .turnSpeed(0.5)
                .lookahead(5.0)
                .maintainHeading(true)
                .start(new Pose2d(depotPosX, -40.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .addGuidePoint(new Pose2d(0.8, -35.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .speed(0.6)
                .addGuidePoint(new Pose2d(0.9, -8.0, (2.0 * Math.PI) - (Math.PI / 2.0)))
                .addTask(() -> {
                    subsystems.intake.run(false, false, false);
                    subsystems.lift.setLinkagePosition(1.0);
                })
                .speed(0.3)
                .end(new Pose2d(-21.0, 4.0, 5.67))
                .build();
    }

}
