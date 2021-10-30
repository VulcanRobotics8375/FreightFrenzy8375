package org.firstinspires.ftc.teamcode.robotcorelib.motion.path;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.PathPoint;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Point;

import java.util.ArrayList;
import java.util.HashMap;

public class Path {

    private final PathPoint start;
    private final ArrayList<PathPoint> guidePoints;
    private final PathPoint end;

    private final HashMap<PathPoint, Runnable> runnableTasks;

    public Path(PathBuilder builder) {
        start = builder.getStartPoint();
        end = builder.getEndPoint();
        guidePoints = builder.getGuidePoints();
        runnableTasks = builder.getTasks();
    }

    public PathPoint getStart() {
        return start;
    }

    public ArrayList<PathPoint> getGuidePoints() {
        return guidePoints;
    }

    public PathPoint getEnd() {
        return end;
    }

    public HashMap<PathPoint, Runnable> getRunnableTasks() {
        return runnableTasks;
    }

    public ArrayList<PathPoint> asList() {
        ArrayList<PathPoint> pathPoints = new ArrayList<>(guidePoints);
        pathPoints.add(0, start);
        pathPoints.add(end);

        return pathPoints;
    }


}
