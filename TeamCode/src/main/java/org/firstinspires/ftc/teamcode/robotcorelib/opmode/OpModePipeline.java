package org.firstinspires.ftc.teamcode.robotcorelib.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.RobotConfig;
import org.firstinspires.ftc.teamcode.robotcorelib.util.AutoTask;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

public abstract class OpModePipeline extends OpMode {
    protected RobotConfig subsystems;
    protected RobotRunMode runMode;

    private volatile boolean stopRequested;

    @Override
    public void init() {
        Robot.init(this);
        subsystems = Robot.getConfiguration();
        if(runMode == RobotRunMode.TELEOP) {
            //teleop specific init
            telemetry.addLine("teleop mode");
        } else if(runMode == RobotRunMode.AUTONOMOUS) {
            //auton specific init
            telemetry.addLine("auton mode");
        }
        telemetry.update();
    }

    public void start() {

    }

    public abstract void loop();


    public void stop() {
        stopRequested = true;
        Robot.stop();
    }

    public boolean isStopRequested() {
        return stopRequested;
    }

    protected void runTask(AutoTask runnable) {
        while(runnable.conditional() && isStopRequested()) {
            Robot.update();
            runnable.run();
        }
    }

}
