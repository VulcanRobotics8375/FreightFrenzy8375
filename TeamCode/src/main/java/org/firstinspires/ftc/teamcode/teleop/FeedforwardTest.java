package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

public class FeedforwardTest extends OpModePipeline {
    private DcMotor motor;
     com.acmerobotics.roadrunner.control.PIDCoefficients coefficients = new PIDCoefficients(0.005, 0.001, 0.001);
    private PIDFController controller = new PIDFController(coefficients, 0.0, 0.0, 0.0);

    public void init(){
        runMode = RobotRunMode.TELEOP;
        super.init();
        motor = hardwareMap.dcMotor.get("lift2");
    }

    public void loop(){
        controller.setTargetPosition(200.0);
        controller.setTargetVelocity(100.0);
        controller.setTargetAcceleration(0);
        controller.setOutputBounds(-0.95, 0.95);
    }
}