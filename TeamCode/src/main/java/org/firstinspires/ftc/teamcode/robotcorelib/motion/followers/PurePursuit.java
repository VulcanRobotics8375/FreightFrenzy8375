package org.firstinspires.ftc.teamcode.robotcorelib.motion.followers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robotcorelib.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.robotcorelib.drive.DriveMode;
import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.math.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.math.SimplePID;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.kinematics.DriveKinematics;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.PathPoint;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Point;

import java.util.ArrayList;
import java.util.Objects;

import static org.firstinspires.ftc.teamcode.robotcorelib.drive.DriveConstants.*;

public class PurePursuit extends Follower {

    public volatile boolean following;
    private SimplePID velocityPid = new SimplePID(1.0, 0.0, 0.0, -1.0, 1.0);
    private SimplePID turnPid = new SimplePID(-2.0, -0.01, 0.0, -1.0, 1.0);

    private LinearOpMode opMode;

    private int pathPointIdx;

    //TODO make these something accessible by the user
    public static final double ALLOWED_POSE_ERROR = 0.6;
    public static final double ALLOWED_HEADING_ERROR = 5.0;
    public static final double POSE_ERROR_GAIN = 0.2;
    public static final double HEADING_ERROR_GAIN = 3.0;

    public PurePursuit() {}

    public PurePursuit(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void followPath(Path path) {
        following = true;

        // base follower
        ArrayList<PathPoint> pathPoints = path.asList();
        PathPoint startPoint = new PathPoint();
        startPoint.setPathPoint(path.getStart());
        PathPoint endPoint = new PathPoint();
        endPoint.setPathPoint(path.getEnd());
        while(following && !opMode.isStopRequested()) {
            Robot.update();
            Pose2d robotPose = Robot.getRobotPose();
            Pose2d robotVel = Robot.getRobotVelocity();

            PathPoint followPoint = findFollowPoint(pathPoints, robotPose, startPoint, endPoint);

            opMode.telemetry.addData("follow point", followPoint.x + ", " + followPoint.y);

            moveToPoint(followPoint, robotPose, robotVel);
            if(Math.hypot(robotPose.getX() - path.getEnd().x, robotPose.getY() - path.getEnd().y) < ALLOWED_POSE_ERROR + 2.0) {
                following = false;
            }

            Runnable task = path.getRunnableTasks().get(path.get(pathPointIdx));
            if(task != null) {
                task.run();
            }

        }

        //mitigate pose error
//        if(Robot.drivetrain.getDriveMode() == DriveMode.MECANUM) {
//            endPoint = pathPoints.get(pathPoints.size() - 1);
//            mitigatePoseError(endPoint);
//        }

        Robot.drivetrain.setPowers(new double[] {0, 0, 0, 0});

    }

    private PathPoint findFollowPoint(ArrayList<PathPoint> path, Pose2d robotPose, PathPoint startPoint, PathPoint endPoint) {
        PathPoint followPoint = path.get(0);

        ArrayList<Point> circleIntersections;
        for (int i = 0; i < path.size() - 1; i++) {
            PathPoint start = path.get(i);
            PathPoint end = path.get(i + 1);

            if(i + 1 == path.size() - 1) {
                if(end.x < start.x) {
                    circleIntersections = MathUtils.lineCircleIntersect(start.toPoint(), end.toPoint(), end.lookahead, new Point(robotPose.getX(), robotPose.getY()), false, true);
                } else {
                    circleIntersections = MathUtils.lineCircleIntersect(start.toPoint(), end.toPoint(), end.lookahead, new Point(robotPose.getX(), robotPose.getY()), true, false);
                }
            } else {
                circleIntersections = MathUtils.lineCircleIntersect(start.toPoint(), end.toPoint(), end.lookahead, new Point(robotPose.getX(), robotPose.getY()), false, true);
            }

            double closestAngle = Double.MAX_VALUE;
            for (Point intersection : circleIntersections) {
                double angle = MathUtils.fullAngleWrap(Math.atan2(intersection.y - robotPose.getY(), intersection.x - robotPose.getX()));
                double relativePointAngle = MathUtils.fullAngleWrap(Math.atan2(end.y - start.y, end.x - start.x));
                double deltaAngle = Math.abs(MathUtils.calcAngularError(relativePointAngle, angle));

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followPoint.setPathPoint(end);
                    followPoint.setPoint(intersection);
                    pathPointIdx = i;
                }

                double maxX = Math.max(start.x, end.x);
                double minX = Math.min(start.x, end.x);
                if(followPoint.x > maxX || followPoint.x < minX) {
                    followPoint.setPoint(end.toPoint());
                }

            }
        }

        //Motion Profile Generation
        // convert continuous, time-variant motion profile to discrete, time-invariant motion profile
        //transfer function-- https://www.desmos.com/calculator/rlv4hdqutl
        double targetVel = MAX_VELOCITY * followPoint.speed;
        double accelDistance = (targetVel*targetVel) / (2.0 * MAX_ACCEL);
        double minSpeed = 0.15;
        double m = (1 - minSpeed) / accelDistance;

        double originalSpeed = followPoint.speed;
        double distanceFromStart = Math.hypot(robotPose.getX() - startPoint.x, robotPose.getY() - startPoint.y);
        double distanceFromEnd = Math.hypot(robotPose.getX() - endPoint.x, robotPose.getY() - endPoint.y);
        if(distanceFromEnd < accelDistance) {
            followPoint.speed *= m * distanceFromEnd;
        }
        else if(distanceFromStart < accelDistance) {
            followPoint.speed *= m * distanceFromStart;
        }
        if(Math.abs(followPoint.speed) < minSpeed) {
            followPoint.speed = minSpeed * Math.signum(originalSpeed);
        }

        return followPoint;
    }


    private void moveToPoint(PathPoint point, Pose2d robotPose, Pose2d robotVelocity) {
        switch (Robot.drivetrain.getDriveMode()) {
            case MECANUM:
                double absoluteAngleToPoint = MathUtils.fullAngleWrap(Math.atan2(robotPose.getY() - point.y, robotPose.getX() - point.x));

                Vector2d poseVelocity = new Vector2d(-Math.cos(absoluteAngleToPoint), Math.sin(absoluteAngleToPoint)).times(point.speed);

                double headingError = MathUtils.calcAngularError(point.theta, robotPose.getHeading());
                opMode.telemetry.addData("heading error", headingError);
                double headingOutput = turnPid.run(headingError);
                Pose2d outputVelocity;
                switch(Robot.drivetrain.getVelocityControlMode()) {
                    case DRIVE_MOTOR_ENCODERS:
                        outputVelocity = new Pose2d(poseVelocity, headingOutput);
                        break;
                    case FEEDFORWARD:
                        //lmao idek how to start ff
                        //TODO figure out feedforward velocity control
                        outputVelocity = new Pose2d(0, 0, 0);
                        break;
                    case PID:
                        //TODO add heading velocity PID to this
                        double velocityNorm = velocityPid.run(poseVelocity.norm(), robotVelocity.vec().norm() / MAX_VELOCITY);
                        Vector2d PidOutputVelocity = new Vector2d(Math.cos(absoluteAngleToPoint), Math.sin(absoluteAngleToPoint)).times(velocityNorm);
                        outputVelocity = new Pose2d(PidOutputVelocity, headingOutput);
                        break;
                    default:
                        outputVelocity = new Pose2d();
                        break;
                }
                double[] powers = DriveKinematics.mecanumFieldVelocityToWheelVelocities(robotPose, outputVelocity);
                Robot.drivetrain.setPowers(powers);
                break;
            case TANK:
            default:
        }
    }

    private void mitigatePoseError(PathPoint pose) {
        Robot.update();
        Pose2d robotPose = Robot.getRobotPose();
        Pose2d robotVelocity;
        double poseError = Math.hypot(pose.x - robotPose.getX(), pose.y - robotPose.getY());
        double headingError = MathUtils.calcAngularError(pose.theta, robotPose.getHeading());
        while((poseError > ALLOWED_POSE_ERROR || headingError > Math.toRadians(ALLOWED_HEADING_ERROR)) && opMode.opModeIsActive()) {
            Robot.update();
            robotPose = Robot.getRobotPose();
            robotVelocity = Robot.getRobotVelocity();
            poseError = Math.hypot(pose.x - robotPose.getX(), pose.y - robotPose.getY());
            headingError = MathUtils.calcAngularError(pose.theta, robotPose.getHeading());
            pose.speed = poseError * POSE_ERROR_GAIN; //pose gain
            pose.turnSpeed = headingError * HEADING_ERROR_GAIN; //heading gain
            moveToPoint(pose, robotPose, robotVelocity);
        }
    }

}
