package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotcorelib.math.PID;
import org.firstinspires.ftc.teamcode.robotcorelib.opmode.OpModePipeline;
import org.firstinspires.ftc.teamcode.robotcorelib.robot.Robot;
import org.firstinspires.ftc.teamcode.robotcorelib.util.RobotRunMode;

@TeleOp
public class PIDTest extends OpModePipeline {
    private DcMotor motor;
    PID pid = new PID(0.005, 0.001, 0, 0.95, -0.95);
    public   void init(){
        runMode = RobotRunMode.TELEOP;
        super.init();
        motor = hardwareMap.dcMotor.get("lift");
    }

    @Override
    public void loop() {
        Robot.update();
        double output = pid.run(200, motor.getCurrentPosition());
        motor.setPower(output);

        telemetry.addData("motor pos", motor.getCurrentPosition());
        telemetry.addData("Integral ", pid.getIntegral());
        telemetry.addData("Integral Error ", pid.getIntegralError());
        telemetry.addData("Controller Output ", pid.getControllerOutput());
        telemetry.addData("Error ", pid.getError());
        telemetry.update();

    }
}
