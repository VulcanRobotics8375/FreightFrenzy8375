package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.HardwarePrecision;

public class Intake extends Subsystem {
    private DcMotor intake;
    private DcMotor transfer;

    private boolean indexer = false;

    private double intakePower;
    private double transferPower;

    public final double INTAKE_POWER = 1;
    public final double TRANSFER_POWER = 1;
    public final int INDEXER_POS = 200;

    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intake");
        transfer = hardwareMap.dcMotor.get("transfer");

        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void run(boolean on, boolean outtake) {
        boolean currentIndex = indexer;

        double intakePower;
        double transferPower;
        int indexerPos = transfer.getCurrentPosition();
        if(on) {
            transferPower = TRANSFER_POWER;
            intakePower = indexer ? 0 : INTAKE_POWER;

        } else if(outtake) {
            intakePower = -INTAKE_POWER;
            transferPower = -TRANSFER_POWER;
        } else {
            intakePower = 0;
            transferPower = 0;
        }

        if(indexerPos > INDEXER_POS && on) {
            indexer = true;
        }
        if(!on) {
            indexer = false;
        }

        if(currentIndex != indexer && indexer) {
            gamepad1.rumble(1.0, 1.0, 500);
            gamepad2.rumble(1.0, 1.0, 500);
        }

        if(MathUtils.shouldHardwareUpdate(intakePower, this.intakePower, HardwarePrecision.LOW)) {
            intake.setPower(intakePower);
        }
        if(MathUtils.shouldHardwareUpdate(transferPower, this.transferPower, HardwarePrecision.LOW)) {
            transfer.setPower(transferPower);
        }

        this.intakePower = intakePower;
        this.transferPower = transferPower;
    }

}
