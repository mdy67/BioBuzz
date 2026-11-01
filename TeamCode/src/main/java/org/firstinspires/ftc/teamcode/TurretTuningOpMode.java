package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Turret PID Tuning", group = "Test")
public class TurretTuningOpMode extends OpMode {

    // FTC Dashboard editable tuning parameters for Target 1, Target 2, Single PID gains, and threshold
    public static double TUNING_TARGET_ANGLE_1 = 0.0;
    public static double TUNING_TARGET_ANGLE_2 = 45.0;
    public static double TUNING_KP = 0.006;
    public static double TUNING_KI = 0.002;
    public static double TUNING_KD = 0.0008;
    public static double TUNING_KS = 0.07;
    public static double TUNING_FULL_POWER_THRESHOLD = 91.0;

    private Robot robot;
    private boolean activeTargetIsTwo = false;
    private boolean lastXButton = false;

    // Rate limiter for gamepad fine adjustments (0.1 degree increments)
    private static class RateLimiter {
        private long pressStartTime = 0;
        private long lastRepeatTime = 0;
        private final long initialDelayMs = 300;
        private final long repeatIntervalMs = 50;

        public boolean shouldTrigger(boolean isPressed) {
            long now = System.currentTimeMillis();
            if (!isPressed) {
                pressStartTime = 0;
                lastRepeatTime = 0;
                return false;
            }

            if (pressStartTime == 0) {
                pressStartTime = now;
                lastRepeatTime = now;
                return true;
            }

            if (now - pressStartTime >= initialDelayMs) {
                if (now - lastRepeatTime >= repeatIntervalMs) {
                    lastRepeatTime = now;
                    return true;
                }
            }

            return false;
        }
    }

    private final RateLimiter leftLimiter = new RateLimiter();
    private final RateLimiter rightLimiter = new RateLimiter();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.turret.state = Turret.State.LOCKED;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Apply FTC Dashboard tuning parameters
        Turret.kP = TUNING_KP;
        Turret.kI = TUNING_KI;
        Turret.kD = TUNING_KD;
        Turret.kS = TUNING_KS;
        Turret.FULL_POWER_THRESHOLD = TUNING_FULL_POWER_THRESHOLD;

        // Toggle between Target Angle 1 and Target Angle 2 when pressing X
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastXButton) {
            activeTargetIsTwo = !activeTargetIsTwo;
        }
        lastXButton = xPressed;

        double currentActiveTarget = activeTargetIsTwo ? TUNING_TARGET_ANGLE_2 : TUNING_TARGET_ANGLE_1;

        // Gamepad fine adjustment (+/- 0.1 degrees per loop when holding dpad left/right or bumpers)
        if (leftLimiter.shouldTrigger(gamepad1.dpad_left)) {
            currentActiveTarget -= 0.03;
            if (activeTargetIsTwo) TUNING_TARGET_ANGLE_2 = currentActiveTarget;
            else TUNING_TARGET_ANGLE_1 = currentActiveTarget;
        }
        if (rightLimiter.shouldTrigger(gamepad1.dpad_right)) {
            currentActiveTarget += 0.03;
            if (activeTargetIsTwo) TUNING_TARGET_ANGLE_2 = currentActiveTarget;
            else TUNING_TARGET_ANGLE_1 = currentActiveTarget;
        }

        // Always in LOCKED state for PID tuning between target angles
        robot.turret.state = Turret.State.LOCKED;
        robot.turret.targetAngle = currentActiveTarget;

        robot.update();

        // Telemetry monitoring
        double currentAngle = robot.turret.getTurretAngleDegrees();
        double constrainedTarget = robot.turret.getClosestValidTarget(robot.turret.targetAngle, currentAngle);
        double error = constrainedTarget - currentAngle;

        telemetry.addData("Active Target", activeTargetIsTwo ? "Target Angle 2" : "Target Angle 1");
        telemetry.addData("Target Angle (deg)", robot.turret.targetAngle);
        telemetry.addData("Constrained Target Angle (deg)", constrainedTarget);
        telemetry.addData("Current Turret Angle (deg)", currentAngle);
        telemetry.addData("Angle Error (deg)", error);
        telemetry.addData("Full Power Threshold", Turret.FULL_POWER_THRESHOLD);
        telemetry.addData("Battery Voltage (V)", robot.turret.getBatteryVoltage());
        telemetry.addData("Turret Servo 1 Power", robot.turret.turretServo1 != null ? robot.turret.turretServo1.getPower() : 0.0);
        telemetry.addData("Turret Servo 2 Power", robot.turret.turretServo2 != null ? robot.turret.turretServo2.getPower() : 0.0);
        telemetry.addData("Turret Min Limit (deg)", Turret.TURRET_MIN_ANGLE);
        telemetry.addData("Turret Max Limit (deg)", Turret.TURRET_MAX_ANGLE);
        telemetry.addData("Loop Time (ms)", robot.loopTimeMs);
        telemetry.addData("Loop Frequency (Hz)", robot.loopHz);
        telemetry.update();
    }
}
