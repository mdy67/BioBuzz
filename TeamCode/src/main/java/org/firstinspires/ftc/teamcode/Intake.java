package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING,
        TRANSFERRING
    }

    public State state = State.IDLE;

    public DcMotor intakeMotor;
    public DcMotor transferMotor;

    public double intakePower = 0.7;
    public double transferIntakePower = 0.25;
    public double transferPower = 0.3; // Transfer power for shooting

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setTransferPower(double power) {
        this.transferPower = Math.max(0.0, Math.min(1.0, power));
    }

    public void adjustTransferPower(double delta) {
        setTransferPower(this.transferPower + delta);
    }

    public void update() {
        switch (state) {
            case INTAKING:
                intakeMotor.setPower(intakePower);
                transferMotor.setPower(-transferIntakePower);
                break;

            case OUTTAKING:
                intakeMotor.setPower(-intakePower);
                transferMotor.setPower(transferIntakePower);
                break;

            case TRANSFERRING:
                intakeMotor.setPower(intakePower);
                transferMotor.setPower(-transferPower); // Uses adjustable transfer power
                break;

            case IDLE:
            default:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                break;
        }
    }
}
