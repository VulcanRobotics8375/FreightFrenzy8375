package org.firstinspires.ftc.teamcode.robotcorelib.motion.followers;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.PathPoint;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Point;

import java.util.ArrayList;

public class PurePursuit extends Follower {

     boolean following;

    public PurePursuit() {}

    public void followPath(Path path) {
        following = true;

        // base follower
        while(following) {
            Robot.update();
            Pose2d robotPose = Robot.getRobotPose();
            Pose2d robotVel = Robot.getRobotVelocity();
            ArrayList<PathPoint> pathPoints = path.asList();

            PathPoint followPoint = findFollowPoint(pathPoints, robotPose);



        }

        //mitigate pose error

    }

    private PathPoint findFollowPoint(ArrayList<PathPoint> path, Pose2d robotPose) {
        PathPoint followPoint = path.get(0);

        ArrayList<Point> circleIntersections;

        for (int i = 0; i < path.size() - 1; i++) {
            PathPoint start = path.get(i);
            PathPoint end = path.get(i + 1);

            if(i + 1 == path.size() - 1 && start.x > end.x) {
                circleIntersections = MathUtils.lineCircleIntersect(start.toPoint(), end.toPoint(), end.lookahead, new Point(robotPose.getX(), robotPose.getY()), true, false);
            } else {
                circleIntersections = MathUtils.lineCircleIntersect(start.toPoint(), end.toPoint(), end.lookahead, new Point(robotPose.getX(), robotPose.getY()), false, true);
            }

            double closestAngle = Double.MAX_VALUE;
            for (Point intersection : circleIntersections) {
                double angle = Math.atan2(intersection.y - robotPose.getY(), intersection.x - robotPose.getX());
                double relativePointAngle = Math.atan2(end.y - start.y, end.x - start.x);
                double deltaAngle = MathUtils.angleWrap(angle - relativePointAngle);

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followPoint.setPathPoint(end);
                    followPoint.setPoint(intersection);
                }

                double maxX = Math.max(start.x, end.x);
                double minX = Math.min(start.x, end.x);
            }

            double maxX = Math.max(start.x, end.x);
            double minX = Math.min(start.x, end.x);


        }

        return followPoint;
    }

}
