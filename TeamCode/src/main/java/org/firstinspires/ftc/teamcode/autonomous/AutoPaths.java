package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robot.FreightFrenzyConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.PurePursuit;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.PathBuilder;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.util.AutoTask;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

public class AutoPaths extends OpModePipeline {

    PurePursuit follower = new PurePursuit();
    FreightFrenzyConfig subsystems = new FreightFrenzyConfig();

    public void init() {
        super.subsystems = subsystems;
        runMode = RobotRunMode.AUTONOMOUS;
        super.init();
    }

    public void start() {}

    @Override
    public void loop() {

        Path path = new PathBuilder()
                .setStartPoint(new Pose2d(0, 0, 0))
                .addGuidePoint(new Pose2d(50, 150, 0))
                .addTask(() -> {
                    //synchronous task goes here
                })
                .setEndPoint(new Pose2d(100, 100, 0))
                .build();
        follower.followPath(path);

        runTask(new AutoTask() {
            @Override
            public boolean conditional() {
                return true;

            }
            @Override
            public void run() {

            }
        });




    }
}
