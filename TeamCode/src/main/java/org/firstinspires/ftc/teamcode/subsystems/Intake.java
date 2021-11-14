package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotcorelib.math.MathUtils;
import org.firstinspires.ftc.teamcode.robotcorelib.util.Subsystem;
import org.firstinspires.ftc.teamcode.robotcorelib.util.hardware.HardwarePrecision;

public class Intake extends Subsystem {
    private DcMotor intake;
    private DcMotor transfer;
    private Servo filter;

    private boolean indexer = false;
    private boolean filterButton = false;
    private double filterOn = -1.0;

    private double intakePower;
    private double transferPower;

    public final double INTAKE_POWER = 1;
    public final double TRANSFER_POWER = 1;
    public final int INDEXER_POS = 200;

    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intake");
        transfer = hardwareMap.dcMotor.get("transfer");
        filter = hardwareMap.servo.get("filter");

        transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void run(boolean on, boolean outtake, boolean filterButton) {
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
         //   gamepad1.rumble(1.0, 1.0, 500);
          //  gamepad2.rumble(1.0, 1.0, 500);
        }

        if(MathUtils.shouldHardwareUpdate(intakePower, this.intakePower, HardwarePrecision.LOW)) {
            intake.setPower(intakePower);
        }
        if(MathUtils.shouldHardwareUpdate(transferPower, this.transferPower, HardwarePrecision.LOW)) {
            transfer.setPower(transferPower);
        }

        if(filterButton && !this.filterButton){
            filterOn *= -1;
            this.filterButton = true;
        }
        if(!filterButton && this.filterButton){
            this.filterButton = false;
        }
        if(filterOn > 0){
            filter.setPosition(0.5);
        }
        if(filterOn < 0){
            filter.setPosition(0.05);
        }

        this.intakePower = intakePower;
        this.transferPower = transferPower;
    }

}
