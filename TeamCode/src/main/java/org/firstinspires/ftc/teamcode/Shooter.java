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
    private DcMotorEx rpmPort;
    public VoltageSensor voltageSensor;

    // Encoder resolution (ticks per rev on flywheel motor)
    public static double TICKS_PER_REV = 28.0;
    public static double MAX_RPM = 6000.0;
    public static double NOMINAL_VOLTAGE = 12.0;

    // Target velocity in RPM and open-loop target power fallback
    public double targetRPM = 0.0;
    public double targetPower = 0.0;
    public boolean useVelocityPID = true;

    // Insanely smooth flywheel tuning parameters optimized for 3000-4000 RPM
    public static double kP = 0.0005;
    public static double kI = 0.0002;
    public static double kD = 0.000008;
    public static double kF = 0.00016;
    public static double kStatic = 0.05;
    public static double MIN_POWER = 0.08;
    public static double ACCEL_THRESHOLD_RPM = 400.0;
    public static double DEADBAND_RPM = 20.0; // Deadband window to prevent hunting/jitter near setpoint

    // Quintic regression coefficients for RPM based on distance to goal:
    // RPM = a*d^5 + b*d^4 + c*d^3 + d*d^2 + e*d + f
    public static double rpmA = 0.0;
    public static double rpmB = 0.0;
    public static double rpmC = 0.00001;
    public static double rpmD = -0.001;
    public static double rpmE = 20.0;
    public static double rpmF = 1500.0;

    private double filteredRPM = 0.0;
    private double currentPowerOutput = 0.0;
    private double integralSum = 0.0;
    private long lastTime = System.currentTimeMillis();

    public Shooter(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheelL");
        rpmPort = hardwareMap.get(DcMotorEx.class, "leftFront"); // port motor encoder for flywheel is plugged in to

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }
    }

    public static double calculateQuinticRPM(double distanceInches) {
        double d = distanceInches;
        return (rpmA * Math.pow(d, 5)) +
               (rpmB * Math.pow(d, 4)) +
               (rpmC * Math.pow(d, 3)) +
               (rpmD * Math.pow(d, 2)) +
               (rpmE * d) +
               rpmF;
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

    public void setPower(double power) {
        this.targetPower = Math.max(0.0, Math.min(1.0, power));
        this.useVelocityPID = false;
    }

    public void adjustRPM(double deltaRPM) {
        setTargetRPM(this.targetRPM + deltaRPM);
    }

    /**
     * Unorthodox, insanely smooth jitter-free velocity PID controller optimized for 3000-4000 RPM.
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
        // Exponential moving average filter on RPM to remove encoder quantization jitter
        filteredRPM = 0.7 * filteredRPM + 0.3 * rawRPM;

        double rpmError = targetRPM - filteredRPM;

        // 1. Fast Acceleration Phase
        if (rpmError > ACCEL_THRESHOLD_RPM) {
            integralSum = 0.0;
            double targetAccelPower = Math.min(1.0, voltageScale);
            // Slew rate limit acceleration
            currentPowerOutput += Math.max(-0.05, Math.min(0.05, targetAccelPower - currentPowerOutput));
            return currentPowerOutput;
        }

        long currentTime = System.currentTimeMillis();
        double elapsedSeconds = (currentTime - lastTime) / 1000.0;
        double dt = (elapsedSeconds <= 0) ? 0.001 : elapsedSeconds;
        lastTime = currentTime;

        // 2. Feedforward + Static Friction Compensation
        double ff = (targetRPM * kF) + (targetRPM > 0 ? kStatic : 0.0);

        // 3. Deadband to prevent hunting / oscillating by 50-60 RPM near setpoint
        double pid;
        if (Math.abs(rpmError) > DEADBAND_RPM) {
            integralSum += rpmError * dt;
            integralSum = Math.max(-200.0, Math.min(200.0, integralSum));
            pid = (kP * rpmError) + (kI * integralSum);
        } else {
            // Inside deadband: freeze integral and suppress proportional jitter
            pid = 0.0;
        }

        double scaledPower = (ff + pid) * voltageScale;
        double desiredPower = Math.max(MIN_POWER, Math.min(1.0, scaledPower));

        // 4. Slew-Rate Limiter for ultra-smooth power transitions (prevents hunting/twitching)
        double maxDeltaPower = 0.03; // max power change per loop
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
