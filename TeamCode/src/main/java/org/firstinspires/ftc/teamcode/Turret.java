package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Turret {

    public enum State {
        IDLE,
        LOCKED,
        TRACKING,
        SOTM // Shoot On The Move
    }

    public enum Goal {
        GOAL_L,
        GOAL_R
    }

    public State state = State.IDLE;
    public Goal currentGoal = Goal.GOAL_R;

    public CRServo turretServo1;
    public CRServo turretServo2;
    public DcMotorEx turretEncoder; // REV Through Bore Encoder plugged into motor port "flywheelR"
    public Servo hoodServo;
    public Servo gateServo;
    public VoltageSensor voltageSensor;

    // Gear Ratio: Encoder attached to 27 tooth gear meshing with 100 tooth turret ring gear
    public static final double TURRET_TO_ENCODER_GEAR_RATIO = 100.0 / 27.0;

    // REV Through Bore Encoder has 8192 counts per revolution
    public static final double TICKS_PER_REV = 8192.0;
    public static final double TICKS_PER_TURRET_DEGREE = (TICKS_PER_REV * TURRET_TO_ENCODER_GEAR_RATIO) / 360.0;

    // Turret rotation limits calculated from encoder max/min ticks (-20526 CCW, 28362 CW) relative to 0 deg init
    public static final double TICK_MIN_LIMIT = -20526.0;
    public static final double TICK_MAX_LIMIT = 28362.0;

    public static double TURRET_MIN_ANGLE = -423.50;
    public static double TURRET_MAX_ANGLE = 156.42;

    // Hood Servo constraints (default 0.5)
    public static final double HOOD_MIN = 0.0;
    public static final double HOOD_MAX = 1.0;
    public double hoodPosition = 0.5;

    // Gate Servo positions
    public double gatePosition = 0.5;

    // Target positions and angles in true field coordinates
    public double targetAngle = 0.0; // Default facing 0 degrees
    public double lockAngle = 0.0;
    public double targetX = 0.0;     // True field target X position (inches)
    public double targetY = 0.0;     // True field target Y position (inches)

    // True field coordinates for Goal L and Goal R [+/- flipped]
    public static double goalLX = 12.0;
    public static double goalLY = 14.0;
    public static double goalRX = -12.0;
    public static double goalRY = 14.0;

    // Simplified SOTM tuning coefficients (based on robot velocity and joystick inputs)
    public static double sotmVelKp = 0.02;
    public static double sotmJoyKp = 1.5;

    // Best Single PID gains & Battery Voltage Compensation parameters
    public static double kP = 0.0007;
    public static double kI = 0.0008;
    public static double kD = 0.00008;
    public static double kS = 0.05; // Static friction compensation at 13.2V
    public static double servoInvert = 1.0; // No servo inversion as requested
    public static double turretBaseOffsetDeg = 180.0; // Tunable base heading offset
    public static double TUNED_VOLTAGE = 13.2;
    public static double FULL_POWER_THRESHOLD = 60.0;

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double filteredDerivative = 0.0;
    private long lastTime = System.currentTimeMillis();

    private int encoderOffset = 0;

    public Turret(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(CRServo.class, "turretServo1");
        turretServo2 = hardwareMap.get(CRServo.class, "turretServo2");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "flywheelR");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        gateServo = hardwareMap.get(Servo.class, "gate");

        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }

        resetEncoder(); // Resets encoder on initialization (pointing towards 0 degrees)
        setGoal(Goal.GOAL_R); // Default to Goal R
    }

    public double getBatteryVoltage() {
        if (voltageSensor != null) {
            double v = voltageSensor.getVoltage();
            if (v > 1.0) return v;
        }
        return TUNED_VOLTAGE;
    }

    public void setGoal(Goal goal) {
        this.currentGoal = goal;
        if (goal == Goal.GOAL_L) {
            setTargetPosition(goalLX, goalLY);
        } else {
            setTargetPosition(goalRX, goalRY);
        }
    }

    public void resetEncoder() {
        if (turretEncoder != null) {
            encoderOffset = turretEncoder.getCurrentPosition();
        }
    }

    public double getTurretAngleDegrees() {
        if (turretEncoder == null) return 0.0;
        int currentTicks = turretEncoder.getCurrentPosition() - encoderOffset;
        return currentTicks / TICKS_PER_TURRET_DEGREE;
    }

    public double getDistanceToGoal(Pose2D rawPose) {
        double robotDriveX = rawPose.getX(DistanceUnit.INCH);
        double robotDriveY = rawPose.getY(DistanceUnit.INCH);
        double robotFieldX = robotDriveY;
        double robotFieldY = -robotDriveX;
        return Math.hypot(targetX - robotFieldX, targetY - robotFieldY);
    }

    public void setLockAngle(double angleDegrees) {
        this.lockAngle = angleDegrees;
        this.targetAngle = angleDegrees;
    }

    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void setHoodPosition(double position) {
        this.hoodPosition = Math.max(HOOD_MIN, Math.min(HOOD_MAX, position));
        if (hoodServo != null) {
            hoodServo.setPosition(this.hoodPosition);
        }
    }

    public void setGateTransferring(boolean transferring) {
        this.gatePosition = transferring ? 0.35 : 0.45;
        if (gateServo != null) {
            gateServo.setPosition(this.gatePosition);
        }
    }

    // Wrap angle error into [-180, 180] degrees
    public static double wrapAngleDegrees(double angle) {
        double wrapped = angle;
        while (wrapped > 180.0) wrapped -= 360.0;
        while (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
    }

    public double getClosestValidTarget(double rawTargetAngle, double currentAngle) {
        double bestTarget = currentAngle;
        double minDiff = Double.MAX_VALUE;

        for (int k = -3; k <= 3; k++) {
            double candidate = rawTargetAngle + (k * 360.0);
            if (candidate >= TURRET_MIN_ANGLE && candidate <= TURRET_MAX_ANGLE) {
                double diff = Math.abs(candidate - currentAngle);
                if (diff < minDiff) {
                    minDiff = diff;
                    bestTarget = candidate;
                }
            }
        }

        if (minDiff == Double.MAX_VALUE) {
            double baseWrapped = currentAngle + wrapAngleDegrees(rawTargetAngle - currentAngle);
            bestTarget = Math.max(TURRET_MIN_ANGLE, Math.min(TURRET_MAX_ANGLE, baseWrapped));
        }

        return bestTarget;
    }

    public double calculateTrackingAngle(Pose2D rawPose) {
        double robotDriveX = rawPose.getX(DistanceUnit.INCH);
        double robotDriveY = rawPose.getY(DistanceUnit.INCH);
        double robotHeadingDeg = rawPose.getHeading(AngleUnit.DEGREES);

        // Convert raw Pinpoint odometry frame to true field frame matching drivetrain x/y localization
        double robotFieldX = robotDriveY;
        double robotFieldY = -robotDriveX;

        double dx = targetX - robotFieldX;
        double dy = targetY - robotFieldY;
        double targetFieldHeadingDeg = Math.toDegrees(Math.atan2(dx, dy));
        return wrapAngleDegrees(targetFieldHeadingDeg + robotHeadingDeg + turretBaseOffsetDeg);
    }

    public double calculateSimplifiedSOTMAngle(Pose2D rawPose, double xVel, double yVel,
                                               double joystickStrafe, double joystickForward) {
        double robotDriveX = rawPose.getX(DistanceUnit.INCH);
        double robotDriveY = rawPose.getY(DistanceUnit.INCH);
        double robotHeadingDeg = rawPose.getHeading(AngleUnit.DEGREES);

        double robotFieldX = robotDriveY;
        double robotFieldY = -robotDriveX;

        double dx = targetX - robotFieldX;
        double dy = targetY - robotFieldY;
        double baseAngleDeg = Math.toDegrees(Math.atan2(dx, dy));
        double relativeAngle = wrapAngleDegrees(baseAngleDeg + robotHeadingDeg + turretBaseOffsetDeg);

        double speedMag = Math.hypot(xVel, yVel);
        double joyMag = Math.hypot(joystickStrafe, joystickForward);
        double leadOffset = (speedMag * sotmVelKp) + (joyMag * sotmJoyKp);

        return wrapAngleDegrees(relativeAngle + leadOffset);
    }

    /**
     * Updates the Single PID loop with Square-Root Shaping and Voltage Compensation.
     */
    public void updatePID() {
        long currentTime = System.currentTimeMillis();
        double elapsedSeconds = (currentTime - lastTime) / 1000.0;
        double dt = (elapsedSeconds <= 0) ? 0.001 : elapsedSeconds;

        double currentAngle = getTurretAngleDegrees();
        double constrainedTarget = getClosestValidTarget(targetAngle, currentAngle);
        double error = constrainedTarget - currentAngle;

        // 1. Deadband: Stop powering completely if error < 1 degree
        if (Math.abs(error) < 1.0) {
            integralSum = 0.0;
            if (turretServo1 != null) turretServo1.setPower(0.0);
            if (turretServo2 != null) turretServo2.setPower(0.0);
            lastError = error;
            lastTime = currentTime;
            return;
        }

        double currentVoltage = getBatteryVoltage();
        double voltageScale = TUNED_VOLTAGE / currentVoltage;

        double rawDerivative = (error - lastError) / dt;
        filteredDerivative = 0.7 * filteredDerivative + 0.3 * rawDerivative;

        double outputPower;
        if (Math.abs(error) > FULL_POWER_THRESHOLD) {
            outputPower = Math.signum(error) * Math.min(1.0, voltageScale);
            integralSum = 0.0;
        } else {
            integralSum += error * dt;
            integralSum = Math.max(-50.0, Math.min(50.0, integralSum));

            double linearPid = (kP * error) + (kI * integralSum) + (kD * filteredDerivative);
            double shapedPid = Math.sqrt(Math.abs(linearPid)) * Math.signum(linearPid);
            
            double effectiveKs = kS * voltageScale;
            double staticFriction = (Math.abs(error) > 0.5) ? effectiveKs * Math.signum(error) : 0.0;

            outputPower = (shapedPid + staticFriction) * voltageScale;
        }

        // Endstop Enforcement
        if (currentAngle <= TURRET_MIN_ANGLE) {
            if (error < 0) outputPower = 0.0;
            if (currentAngle < TURRET_MIN_ANGLE - 1.0) outputPower = 0.3;
        } else if (currentAngle >= TURRET_MAX_ANGLE) {
            if (error > 0) outputPower = 0.0;
            if (currentAngle > TURRET_MAX_ANGLE + 1.0) outputPower = -0.3;
        }

        outputPower = Math.max(-1.0, Math.min(1.0, outputPower)) * servoInvert;

        if (turretServo1 != null) turretServo1.setPower(outputPower);
        if (turretServo2 != null) turretServo2.setPower(outputPower);

        lastError = error;
        lastTime = currentTime;
    }

    public void update() {
        if (hoodServo != null) hoodServo.setPosition(hoodPosition);
        if (gateServo != null) gateServo.setPosition(gatePosition);

        switch (state) {
            case LOCKED:
            case TRACKING:
            case SOTM:
                updatePID();
                break;
            case IDLE:
            default:
                if (turretServo1 != null) turretServo1.setPower(0.0);
                if (turretServo2 != null) turretServo2.setPower(0.0);
                break;
        }
    }

    public void updateTracking(Pose2D rawPose) {
        this.targetAngle = calculateTrackingAngle(rawPose);
        this.state = State.TRACKING;
        updatePID();
    }

    public void updateSOTM(Pose2D rawPose, double xVel, double yVel, double joystickStrafe, double joystickForward) {
        this.targetAngle = calculateSimplifiedSOTMAngle(rawPose, xVel, yVel, joystickStrafe, joystickForward);
        this.state = State.SOTM;
        updatePID();
    }

    public void updateSOTM(Pose2D rawPose, double xVel, double yVel, double headingVelDeg,
                           double joystickStrafe, double joystickForward, double joystickTurn) {
        updateSOTM(rawPose, xVel, yVel, joystickStrafe, joystickForward);
    }
}
