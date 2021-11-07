package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;

public class Intake extends Subsystem {
    private DcMotor intake;
    private DcMotor transfer;

    private boolean indexer = false;

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
    }


    public void run(boolean on) {
        double intakePower;
        double transferPower;
        boolean currentIndex = indexer;
        int indexerPos = transfer.getCurrentPosition();
        if(on) {
            transferPower = TRANSFER_POWER;
            intakePower = indexer ? 0 : INTAKE_POWER;

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
            gamepad1.rumble(500);
            gamepad2.rumble(500);
        }

//        telemetry.addData("indexing", indexer);
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

    }

}
