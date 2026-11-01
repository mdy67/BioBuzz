package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class Shooter {

    public enum State {
        IDLE,
        TESTING,
        SHOOTING
    }

    public State state = State.IDLE;

    public DcMotorEx flywheel;
    private final DcMotorEx rpmPort;
    public VoltageSensor voltageSensor;

    // Encoder resolution (ticks per rev on flywheel motor)
    public static double TICKS_PER_REV = 28.0;
    public static double MAX_RPM = 3600.0;
    public static double NOMINAL_VOLTAGE = 12.0;

    // Target velocity in RPM and open-loop target power fallback
    public double targetRPM = 0.0;
    public double targetPower = 0.0;
    public boolean useVelocityPID = true;

    // Smooth flywheel tuning parameters optimized for 3000-4000 RPM
    public static double kP = 0.0005;
    public static double kI = 0.0002;
    public static double kD = 0.000008;
    public static double kF = 0.00016;
    public static double kStatic = 0.05;
    public static double MIN_POWER = 0.08;
    public static double ACCEL_THRESHOLD_RPM = 800.0;
    public static double DEADBAND_RPM = 20.0;

    // Quadratic regression coefficients for RPM based on distance to goal (inches)
    public static double quadA = -0.00968326;
    public static double quadB = 1.15485;
    public static double quadC = 3.01563;

    private double filteredRPM = 0.0;
    private double currentPowerOutput = 0.0;
    private double integralSum = 0.0;
    private long lastTime = System.currentTimeMillis();

    public Shooter(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheelL");
        rpmPort = hardwareMap.get(DcMotorEx.class, "leftFront");

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }
    }

    /**
     * Calculates target RPM using quadratic regression based on distance to goal (inches).
     */
    public static double calculateRpm(double distanceInches) {
        double yDiv100 = (quadA * distanceInches * distanceInches) + (quadB * distanceInches) + quadC;
        return yDiv100 * 100.0;
    }

    public double getBatteryVoltage() {
        if (voltageSensor != null) {
            double v = voltageSensor.getVoltage();
            if (v > 1.0) return v;
        }
        return NOMINAL_VOLTAGE;
    }

    public double getCurrentRPM() {
        if (rpmPort == null) return 0.0;
        double currentTicksPerSec = rpmPort.getVelocity();
        return (currentTicksPerSec * 60.0) / TICKS_PER_REV;
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = Math.max(0.0, Math.min(MAX_RPM, rpm));
        this.useVelocityPID = true;
    }

    public void adjustRPM(double deltaRPM) {
        setTargetRPM(this.targetRPM + deltaRPM);
    }

    /**
     * Velocity PID controller optimized for smooth jitter-free flywheel response at 3000-4000 RPM.
     */
    public double calculateVelocityPower() {
        if (targetRPM <= 0) {
            integralSum = 0.0;
            currentPowerOutput = 0.0;
            filteredRPM = 0.0;
            return 0.0;
        }

        double currentVoltage = getBatteryVoltage();
        double voltageScale = NOMINAL_VOLTAGE / currentVoltage;

        double rawRPM = getCurrentRPM();
        filteredRPM = 0.7 * filteredRPM + 0.3 * rawRPM;

        double rpmError = targetRPM - filteredRPM;

        if (rpmError > ACCEL_THRESHOLD_RPM) {
            integralSum = 0.0;
            double targetAccelPower = Math.min(1.0, voltageScale);
            currentPowerOutput += Math.max(-0.05, Math.min(0.05, targetAccelPower - currentPowerOutput));
            return currentPowerOutput;
        }

        long currentTime = System.currentTimeMillis();
        double elapsedSeconds = (currentTime - lastTime) / 1000.0;
        double dt = (elapsedSeconds <= 0) ? 0.001 : elapsedSeconds;
        lastTime = currentTime;

        double ff = (targetRPM * kF) + (targetRPM > 0 ? kStatic : 0.0);

        double pid;
        if (Math.abs(rpmError) > DEADBAND_RPM) {
            integralSum += rpmError * dt;
            integralSum = Math.max(-200.0, Math.min(200.0, integralSum));
            pid = (kP * rpmError) + (kI * integralSum);
        } else {
            pid = 0.0;
        }

        double scaledPower = (ff + pid) * voltageScale;
        double desiredPower = Math.max(MIN_POWER, Math.min(1.0, scaledPower));

        double maxDeltaPower = 0.03;
        currentPowerOutput += Math.max(-maxDeltaPower, Math.min(maxDeltaPower, desiredPower - currentPowerOutput));

        return currentPowerOutput;
    }

    public void update() {
        switch (state) {
            case TESTING:
            case SHOOTING:
                double powerToApply = useVelocityPID ? calculateVelocityPower() : targetPower;
                flywheel.setPower(powerToApply);
                break;

            case IDLE:
            default:
                flywheel.setPower(0.0);
                integralSum = 0.0;
                filteredRPM = 0.0;
                currentPowerOutput = 0.0;
                break;
        }
    }
}
