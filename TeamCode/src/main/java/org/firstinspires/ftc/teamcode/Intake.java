package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Intake {

    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING,
        TRANSFERRING
    }

    public State state = State.IDLE;
    private State lastState = State.IDLE;
    private long transferStartTime = 0;

    public DcMotor intakeMotor;
    public DcMotor transferMotor;
    public VoltageSensor voltageSensor;

    public double intakePower = 0.7;
    public double transferIntakePower = 0.25;
    public double transferPower = 0.5; // Transfer power for shooting
    public static double TUNED_VOLTAGE = 13.0; // Tuned baseline voltage

    // Tunable dynamic transfer ramp-up parameters
    public static double transferRampInitialScale = 0.2;
    public static double transferRampFinalScale = 1.0;
    public static double transferRampDurationSec = 0.75;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }
    }

    public double getBatteryVoltage() {
        if (voltageSensor != null) {
            double v = voltageSensor.getVoltage();
            if (v > 1.0) return v;
        }
        return TUNED_VOLTAGE;
    }

    public void setTransferPower(double power) {
        this.transferPower = Math.max(0.0, Math.min(1.0, power));
    }

    public void adjustTransferPower(double delta) {
        setTransferPower(this.transferPower + delta);
    }

    public void update() {
        // Track transition into TRANSFERRING state
        if (state == State.TRANSFERRING && lastState != State.TRANSFERRING) {
            transferStartTime = System.currentTimeMillis();
        }
        lastState = state;

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
                // Time-scaled dynamic power ramp-up
                double elapsedSeconds = (System.currentTimeMillis() - transferStartTime) / 1000.0;
                double rampProgress = (transferRampDurationSec <= 0) ? 1.0 : Math.min(1.0, elapsedSeconds / transferRampDurationSec);
                double rampScale = transferRampInitialScale + (transferRampFinalScale - transferRampInitialScale) * rampProgress;

                // Scale transfer power with battery voltage compensation relative to tuned baseline
                double voltageScale = TUNED_VOLTAGE / getBatteryVoltage();
                double scaledTransferPower = Math.min(1.0, transferPower * voltageScale * rampScale);
                double scaledIntakePower = intakePower * 1.3 * rampScale;

                intakeMotor.setPower(scaledIntakePower);
                transferMotor.setPower(-scaledTransferPower);
                break;

            case IDLE:
            default:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                break;
        }
    }
}
