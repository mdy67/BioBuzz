package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {

    private Robot robot;
    private boolean lastXButton = false;
    private boolean lastAButton = false;
    private boolean lastLeftTrigger = false;
    private boolean sotmMode = false;

    private double flywheelRpmOffset = -150.0;
    private double turretAngleOffset = 0.0;

    // Rate limiter helper for dpad adjustments
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

    private final RateLimiter dpadUpLimiter = new RateLimiter();
    private final RateLimiter dpadDownLimiter = new RateLimiter();
    private final RateLimiter dpadLeftLimiter = new RateLimiter();
    private final RateLimiter dpadRightLimiter = new RateLimiter();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.drivetrain.setPosition(0.0, 0.0, 90.0); // Reset position to x:0, y:0, heading:90 at init
        robot.turret.setGoal(Turret.Goal.GOAL_R); // Default Goal R
        robot.turret.state = Turret.State.LOCKED;
    }

    @Override
    public void loop() {
        // 1. Drivetrain control (Pinpoint 1) with reversed forward/backwards and right trigger slow mode (0.3x)
        double strafe = gamepad1.left_stick_x;
        double forward = gamepad1.left_stick_y; // Reversed: positive stick forward
        double turn = gamepad1.right_stick_x * 0.85;

        double speedMultiplier = (gamepad1.right_trigger > 0.1) ? 0.3 : 1.0;

        robot.drivetrain.driveTeleOp(
                strafe * speedMultiplier,
                forward * speedMultiplier,
                turn * speedMultiplier
        );

        // Toggle SOTM vs Tracking with X button
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastXButton) {
            sotmMode = !sotmMode;
        }
        lastXButton = xPressed;

        robot.turret.state = sotmMode ? Turret.State.SOTM : Turret.State.TRACKING;

        // Reset Drivetrain Position & Heading on A Button press (x: 0, y: -65, heading: 90)
        boolean aPressed = gamepad1.a;
        if (aPressed && !lastAButton) {
            robot.drivetrain.setPosition(0.0, -65.0, 90.0);
        }
        lastAButton = aPressed;

        // Toggle Goal R vs Goal L targeting with Left Trigger press
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.1;
        if (leftTriggerPressed && !lastLeftTrigger) {
            if (robot.turret.currentGoal == Turret.Goal.GOAL_R) {
                robot.turret.setGoal(Turret.Goal.GOAL_L);
            } else {
                robot.turret.setGoal(Turret.Goal.GOAL_R);
            }
        }
        lastLeftTrigger = leftTriggerPressed;

        // Turret Angle Offset adjustment via Dpad Left / Right (1 degree per press/hold)
        if (dpadLeftLimiter.shouldTrigger(gamepad1.dpad_left)) {
            turretAngleOffset -= 1.0;
            Turret.turretBaseOffsetDeg = turretAngleOffset;
        }
        if (dpadRightLimiter.shouldTrigger(gamepad1.dpad_right)) {
            turretAngleOffset += 1.0;
            Turret.turretBaseOffsetDeg = turretAngleOffset;
        }

        // Flywheel RPM offset adjustment via Dpad Up / Down
        if (dpadUpLimiter.shouldTrigger(gamepad1.dpad_up)) {
            flywheelRpmOffset += 30.0;
        }
        if (dpadDownLimiter.shouldTrigger(gamepad1.dpad_down)) {
            flywheelRpmOffset -= 30.0;
        }

        // 2. Shooter ALWAYS tracks the goal (Flywheel & Turret) using Quadratic Regression
        double distanceToGoal = robot.turret.getDistanceToGoal(robot.drivetrain.robotPose);
       // double targetRPM = Shooter.calculateRpm(distanceToGoal) + flywheelRpmOffset;

      //  robot.shooter.setTargetRPM(targetRPM);
        robot.shooter.setTargetRPM(3375 + flywheelRpmOffset);
        robot.shooter.state = Shooter.State.SHOOTING;

        // 3. Intake and Transfer/Shoot controls (Bumpers)
        // - Left bumper: Intake
        // - Right bumper: Transfer and shoot wiffle balls (opens gate)
        boolean isTransferring = gamepad1.right_bumper;
        robot.turret.setGateTransferring(isTransferring);

        if (isTransferring) {
            robot.intake.state = Intake.State.TRANSFERRING;
        } else if (gamepad1.left_bumper) {
            robot.intake.state = Intake.State.INTAKING;
        } else if (gamepad1.left_trigger > 0.1) {

        } else {
            robot.intake.state = Intake.State.IDLE;
        }

        // 4. Turret tracking / SOTM update
        if (robot.turret.state == Turret.State.SOTM) {
            robot.turret.updateSOTM(
                    robot.drivetrain.robotPose,
                    robot.drivetrain.XVel, robot.drivetrain.YVel,
                    strafe, forward
            );
        } else if (robot.turret.state == Turret.State.TRACKING) {
            robot.turret.updateTracking(robot.drivetrain.robotPose);
        }

        // 5. Periodic update of all subsystems (Pinpoint 1)
        robot.update();

        // Telemetry (including Field X, Y, and Heading)
        telemetry.addData("Field X Position (in)", robot.drivetrain.x);
        telemetry.addData("Field Y Position (in)", robot.drivetrain.y);
        telemetry.addData("Field Heading (deg)", robot.drivetrain.head);
        telemetry.addData("Active Goal", robot.turret.currentGoal);
        telemetry.addData("Turret Mode", sotmMode ? "SOTM" : "Tracking");
        telemetry.addData("Distance to Goal (in)", distanceToGoal);
        telemetry.addData("Hood Position", robot.turret.hoodPosition);
        telemetry.addData("Turret Angle Offset (deg)", turretAngleOffset);
        telemetry.addData("Flywheel RPM Offset", flywheelRpmOffset);
       // telemetry.addData("Target RPM (Quadratic + Offset)", targetRPM);
        telemetry.addData("Shooter Current RPM", robot.shooter.getCurrentRPM());
        telemetry.addData("Intake State", robot.intake.state);
        telemetry.addData("Gate Position", robot.turret.gatePosition);
        telemetry.addData("Turret Angle (deg)", robot.turret.getTurretAngleDegrees());
        telemetry.addData("Loop Frequency (Hz)", robot.loopHz);
        telemetry.update();
    }
}
