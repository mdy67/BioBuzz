package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Turret {

    public enum State {
        IDLE,
        LOCKED,
        TRACKING,
        SOTM // Shoot On The Move
    }

    public State state = State.IDLE;

    public CRServo turretServo;
    public DcMotorEx turretEncoder; // REV Through Bore Encoder plugged into intake motor port ("intake")
    public Servo hoodServo;
    public Servo gateServo;

    // Gear Ratio: 60t (servo) -> 20t -> 27t (shaft) -> 100t (turret)
    public static final double SERVO_GEAR = 60.0;
    public static final double STAGE1_GEAR = 20.0;
    public static final double STAGE2_GEAR = 27.0;
    public static final double TURRET_GEAR = 100.0;
    public static final double GEAR_RATIO = (SERVO_GEAR / STAGE1_GEAR) * (STAGE2_GEAR / TURRET_GEAR); // 0.81

    // REV Through Bore Encoder has 8192 counts per revolution
    public static final double TICKS_PER_REV = 8192.0;
    public static final double TICKS_PER_TURRET_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    // Hood Servo constraints (default 0.5)
    public static final double HOOD_MIN = 0.0;
    public static final double HOOD_MAX = 1.0;
    public double hoodPosition = 0.5;

    // Gate Servo positions
    public double gatePosition = 0.5;

    // Target positions and angles
    public double targetAngle = 0.0; // In degrees
    public double lockAngle = 0.0;   // In degrees
    public double targetX = 0.0;     // Field target X position (inches)
    public double targetY = 0.0;     // Field target Y position (inches)

    // Shot speed for Shoot On The Move time-of-flight estimation (inches/sec)
    public double shotVelocity = 300.0;

    // PID gains for CR Servo position control
    public static double kP = 0.02;
    public static double kI = 0.0001;
    public static double kD = 0.001;

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private long lastTime = System.currentTimeMillis();

    private int encoderOffset = 0;

    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "intake");
        hoodServo = hardwareMap.get(Servo.class, "hood");
        gateServo = hardwareMap.get(Servo.class, "gate");
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

    public void setLockAngle(double angleDegrees) {
        this.lockAngle = angleDegrees;
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

    public void adjustHoodPosition(double delta) {
        setHoodPosition(this.hoodPosition + delta);
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

    /**
     * Calculates the required turret angle for TRACKING state using Pose2D.
     */
    public double calculateTrackingAngle(Pose2D robotPose) {
        double robotX = robotPose.getX(DistanceUnit.INCH);
        double robotY = robotPose.getY(DistanceUnit.INCH);
        double robotHeadingDeg = robotPose.getHeading(AngleUnit.DEGREES);

        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double targetFieldHeadingRad = Math.atan2(dy, dx);
        double targetFieldHeadingDeg = Math.toDegrees(targetFieldHeadingRad);
        return wrapAngleDegrees(targetFieldHeadingDeg - robotHeadingDeg);
    }

    /**
     * Calculates the predicted turret angle for SOTM (Shoot On The Move) state using Pose2D.
     * Incorporates robot field pose, velocity (xVel, yVel, headingVel),
     * and joystick inputs to lead the target.
     */
    public double calculateSOTMAngle(Pose2D robotPose,
                                     double xVel, double yVel, double headingVelDeg,
                                     double joystickStrafe, double joystickForward, double joystickTurn) {
        double robotX = robotPose.getX(DistanceUnit.INCH);
        double robotY = robotPose.getY(DistanceUnit.INCH);
        double robotHeadingDeg = robotPose.getHeading(AngleUnit.DEGREES);

        double dx = targetX - robotX;
        double dy = targetY - robotY;
        double distance = Math.hypot(dx, dy);

        // Estimate time of flight for shot
        double timeOfFlight = (shotVelocity > 0) ? (distance / shotVelocity) : 0.0;

        // Effective robot velocity incorporating measured velocity and joystick inputs
        double effectiveXVel = xVel + (joystickStrafe * 10.0);
        double effectiveYVel = yVel + (joystickForward * 10.0);
        double effectiveHeadingVel = headingVelDeg + (joystickTurn * 10.0);

        // Predict future robot position and heading at release time
        double predictedRobotX = robotX + effectiveXVel * timeOfFlight;
        double predictedRobotY = robotY + effectiveYVel * timeOfFlight;
        double predictedRobotHeading = robotHeadingDeg + effectiveHeadingVel * timeOfFlight;

        // Calculate angle to target from predicted robot position
        double predDx = targetX - predictedRobotX;
        double predDy = targetY - predictedRobotY;
        double predFieldHeadingDeg = Math.toDegrees(Math.atan2(predDy, predDx));

        return wrapAngleDegrees(predFieldHeadingDeg - predictedRobotHeading);
    }

    /**
     * Updates the PID loop to drive the CR Servo towards targetAngle.
     */
    public void updatePID() {
        long currentTime = System.currentTimeMillis();
        double elapsedSeconds = (currentTime - lastTime) / 1000.0;
        double dt = (elapsedSeconds <= 0) ? 0.001 : elapsedSeconds;

        double currentAngle = getTurretAngleDegrees();
        double error = wrapAngleDegrees(targetAngle - currentAngle);

        integralSum += error * dt;
        // Anti-windup clamping for integral sum
        integralSum = Math.max(-100.0, Math.min(100.0, integralSum));

        double derivative = (error - lastError) / dt;

        double outputPower = (kP * error) + (kI * integralSum) + (kD * derivative);
        outputPower = Math.max(-1.0, Math.min(1.0, outputPower));

        if (turretServo != null) {
            turretServo.setPower(outputPower);
        }

        lastError = error;
        lastTime = currentTime;
    }

    /**
     * Main update loop for Turret.
     */
    public void update() {
        if (hoodServo != null) {
            hoodServo.setPosition(hoodPosition);
        }
        if (gateServo != null) {
            gateServo.setPosition(gatePosition);
        }

        switch (state) {
            case LOCKED:
                targetAngle = lockAngle;
                updatePID();
                break;

            case TRACKING:
            case SOTM:
                updatePID();
                break;

            case IDLE:
            default:
                if (turretServo != null) {
                    turretServo.setPower(0.0);
                }
                break;
        }
    }

    public void updateTracking(Pose2D robotPose) {
        this.targetAngle = calculateTrackingAngle(robotPose);
        this.state = State.TRACKING;
        updatePID();
    }

    public void updateSOTM(Pose2D robotPose,
                           double xVel, double yVel, double headingVelDeg,
                           double joystickStrafe, double joystickForward, double joystickTurn) {
        this.targetAngle = calculateSOTMAngle(robotPose,
                                              xVel, yVel, headingVelDeg,
                                              joystickStrafe, joystickForward, joystickTurn);
        this.state = State.SOTM;
        updatePID();
    }
}
