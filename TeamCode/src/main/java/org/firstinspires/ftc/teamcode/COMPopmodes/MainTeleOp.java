package org.firstinspires.ftc.teamcode.COMPopmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.DataTransfer;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {

    private Robot robot;
    private boolean lastXButton = false;
    private boolean lastYButton = false;
    private boolean lastAButton = false;
    private boolean lastBButton = false;
    private boolean lastLeftTrigger = false;
    private boolean teleopStarted = false;
    private boolean recoveryMode = false;

    private double flywheelRpmOffset = -570.0;
    private double turretAngleOffsetGoalL = 0.0;
    private double turretAngleOffsetGoalR = 0.0;

    // Rate limiter helper for dpad adjustments
    private static class RateLimiter {
        private long pressStartTime = 0;
        private long lastRepeatTime = 0;
        private final long initialDelayMs = 120;
        private final long repeatIntervalMs = 15;

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
        DataTransfer.teleopInit(robot); // Pull position & turret angle from Autonomous if run
        robot.turret.setGoal(Turret.Goal.GOAL_R); // Default Goal R
        robot.turret.state = Turret.State.IDLE;   // Default turret state is IDLE
        robot.shooter.state = Shooter.State.IDLE;
        teleopStarted = false;
    }

    @Override
    public void init_loop() {
        DataTransfer.teleopInit(robot); // Continuously sync transferred position during init
        robot.drivetrain.update();      // Update odometry so telemetry displays live values

        telemetry.addData("Status", "Init Loop: Waiting for Start");
        telemetry.addData("DataTransfer HasRunAuto", DataTransfer.hasRunAuto);
        telemetry.addData("Transferred X", DataTransfer.autoX);
        telemetry.addData("Transferred Y", DataTransfer.autoY);
        telemetry.addData("Transferred Heading", DataTransfer.autoHeading);
        telemetry.addData("Transferred Turret Encoder Offset", DataTransfer.autoEncoderOffset);
        telemetry.addData("Live X Position (in)", robot.drivetrain.x);
        telemetry.addData("Live Y Position (in)", robot.drivetrain.y);
        telemetry.addData("Live Heading (deg)", robot.drivetrain.head);
        telemetry.addData("Active Goal", robot.turret.currentGoal);
        telemetry.update();
    }

    public double getActiveTurretOffset() {
        return (robot.turret.currentGoal == Turret.Goal.GOAL_L) ? turretAngleOffsetGoalL : turretAngleOffsetGoalR;
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

        // Toggle teleop start / SOTM vs Tracking with X button
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastXButton) {
            if (!teleopStarted) {
                teleopStarted = true;
            } else {
                recoveryMode = !recoveryMode;
            }
        }
        lastXButton = xPressed;

        robot.turret.state = recoveryMode || !teleopStarted ? Turret.State.IDLE : Turret.State.TRACKING;

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

        long now = System.currentTimeMillis();

        // Goal-Specific Turret Angle Offset adjustment via Dpad Left / Right
        // 3 degrees each time during IDLE mode, 1 degree during tracking
        double angleStep = (robot.turret.state == Turret.State.IDLE) ? 3.0 : 1.0;

        if (dpadLeftLimiter.shouldTrigger(gamepad1.dpad_left)) {
            if (robot.turret.currentGoal == Turret.Goal.GOAL_L) {
                turretAngleOffsetGoalL -= angleStep;
            } else {
                turretAngleOffsetGoalR -= angleStep;
            }
        }
        if (dpadRightLimiter.shouldTrigger(gamepad1.dpad_right)) {
            if (robot.turret.currentGoal == Turret.Goal.GOAL_L) {
                turretAngleOffsetGoalL += angleStep;
            } else {
                turretAngleOffsetGoalR += angleStep;
            }
        }

        // Apply active goal turret base offset
        Turret.turretBaseOffsetDeg = getActiveTurretOffset();

        // Flywheel RPM offset adjustment via Dpad Up / Down
        if (dpadUpLimiter.shouldTrigger(gamepad1.dpad_up)) {
            flywheelRpmOffset += 30.0;
        }
        if (dpadDownLimiter.shouldTrigger(gamepad1.dpad_down)) {
            flywheelRpmOffset -= 30.0;
        }

        // 2. Shooter sets target RPM (3375 + flywheelRpmOffset) + addition of (rampscale * 0.5) power
        double distanceToGoal = robot.turret.getDistanceToGoal(robot.drivetrain.getLocalizedPose());
        double targetRPM = 3375.0 + flywheelRpmOffset;

        if (teleopStarted) {
            robot.shooter.setTargetRPM(targetRPM);
            robot.shooter.state = Shooter.State.SHOOTING;
        }
        robot.shooter.powerBoost = robot.intake.rampScale * 0.5;

        // 3. Intake and Transfer/Shoot controls (Bumpers)
        boolean isTransferring = gamepad1.right_bumper;
        robot.turret.setGateTransferring(isTransferring);

        if (isTransferring) {
            robot.intake.state = Intake.State.TRANSFERRING;
        } else if (gamepad1.left_bumper) {
            robot.intake.rampScale = 0;
            robot.intake.state = Intake.State.INTAKING;
        } else {
            robot.intake.state = Intake.State.IDLE;
            robot.intake.rampScale = 0;
        }

// Toggle flower servo between flowerUp and flowerDown with Button B
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastBButton) {
            if (robot.intake.flowerUp) {
                robot.intake.setFlowerDown();
            } else {
                robot.intake.setFlowerUp();
            }
        }
        lastBButton = bPressed;

        if (robot.turret.state == Turret.State.TRACKING) {
            robot.turret.updateTracking(robot.drivetrain.robotPose);
        }

        // 5. Periodic update of all subsystems (Pinpoint 1)
        robot.update();

        // Telemetry
        telemetry.addData("Teleop Started?", teleopStarted);
        telemetry.addData("Field X Position (in)", robot.drivetrain.x);
        telemetry.addData("Field Y Position (in)", robot.drivetrain.y);
        telemetry.addData("Field Heading (deg)", robot.drivetrain.head);
        telemetry.addData("Active Goal", robot.turret.currentGoal);
        telemetry.addData("Turret Mode", recoveryMode ? "SOTM" : "Tracking");
        telemetry.addData("Distance to Goal (in)", distanceToGoal);
        telemetry.addData("Hood Position", robot.turret.hoodPosition);
        telemetry.addData("Active Goal Turret Offset (deg)", getActiveTurretOffset());
        telemetry.addData("Flywheel RPM Offset", flywheelRpmOffset);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Shooter Current RPM", robot.shooter.getCurrentRPM());
        telemetry.addData("Intake State", robot.intake.state);
        telemetry.addData("Gate Position", robot.turret.gatePosition);
        telemetry.addData("Flower Position", robot.intake.flowerServo != null ? robot.intake.flowerServo.getPosition() : "N/A");
        telemetry.addData("Turret Angle (deg)", robot.turret.getTurretAngleDegrees());
        telemetry.addData("Loop Frequency (Hz)", robot.loopHz);
        telemetry.update();
    }
}
