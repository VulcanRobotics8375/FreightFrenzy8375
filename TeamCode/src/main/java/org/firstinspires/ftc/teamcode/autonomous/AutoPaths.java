package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.robotcorelib.motion.followers.PurePursuit;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;
import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.PathBuilder;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.util.AutoTask;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

public class AutoPaths extends OpModePipeline {

    PurePursuit follower = new PurePursuit();

    public void init() {
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
                    subsystems.intake.run(true, false, false);
                })
                .setEndPoint(new Pose2d(100, 100, 0))
                .build();
        follower.followPath(path);

        runTask(new AutoTask() {
            @Override
            public boolean conditional() {

            }
            @Override
            public void run() {

            }
        });




    }
}
