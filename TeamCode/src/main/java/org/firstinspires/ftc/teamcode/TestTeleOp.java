package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test TeleOp", group = "TeleOp")
public class TestTeleOp extends OpMode {

    private Robot robot;

    // Button rate limiter helper for dpad/buttons single press vs continuous hold
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
    private final RateLimiter hoodUpLimiter = new RateLimiter();
    private final RateLimiter hoodDownLimiter = new RateLimiter();

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.turret.setHoodPosition(0.5); // Default hood position
    }

    @Override
    public void loop() {
        // 1. Drivetrain teleop control with slow mode (30% speed / 0.3x) when right trigger is held
        double strafe = gamepad1.left_stick_x;
        double forward = gamepad1.left_stick_y; // Stick up is negative in FTC
        double turn = gamepad1.right_stick_x;

        double speedMultiplier = (gamepad1.right_trigger > 0.1) ? 0.3 : 1.0;

        robot.drivetrain.driveTeleOp(
                strafe * speedMultiplier,
                forward * speedMultiplier,
                turn * speedMultiplier
        );

        // 2. Shooter power adjustment (Dpad Up / Dpad Down in increments of 0.01 with holding rate limiter)
        if (dpadUpLimiter.shouldTrigger(gamepad1.dpad_up)) {
            robot.shooter.adjustPower(0.01);
        }
        if (dpadDownLimiter.shouldTrigger(gamepad1.dpad_down)) {
            robot.shooter.adjustPower(-0.01);
        }

        robot.shooter.state = Shooter.State.TESTING;

        // 3. Hood position adjustment (Y and A buttons in increments of 0.01 with holding rate limiter)
        if (hoodUpLimiter.shouldTrigger(gamepad1.y)) {
            robot.turret.setHoodPosition(robot.turret.hoodPosition + 0.01);
        }
        if (hoodDownLimiter.shouldTrigger(gamepad1.a)) {
            robot.turret.setHoodPosition(robot.turret.hoodPosition - 0.01);
        }

        // 4. Turret completely idle
        robot.turret.state = Turret.State.IDLE;

        // 5. Intake, Transfer, and Gate control (Right bumper for transferring moves Gate to 0.9, otherwise 0.5)
        boolean isTransferring = gamepad1.right_bumper;
        robot.turret.setGateTransferring(isTransferring);

        if (isTransferring) {
            robot.intake.state = Intake.State.TRANSFERRING;
        } else if (gamepad1.left_bumper) {
            robot.intake.state = Intake.State.INTAKING;
        } else if (gamepad1.left_trigger > 0.1) {
            robot.intake.state = Intake.State.OUTTAKING;
        } else {
            robot.intake.state = Intake.State.IDLE;
        }

        // 6. Periodic update of all subsystems
        robot.update();

        // Telemetry
        telemetry.addData("Shooter Power", robot.shooter.targetPower);
        telemetry.addData("Shooter State", robot.shooter.state);
        telemetry.addData("Hood Position", robot.turret.hoodPosition);
        telemetry.addData("Gate Position", robot.turret.gatePosition);
        telemetry.addData("Intake State", robot.intake.state);
        telemetry.addData("Turret State", robot.turret.state);
        telemetry.addData("Slow Mode (Right Trigger)", gamepad1.right_trigger > 0.1 ? "Active (30%)" : "Normal");
        telemetry.addData("Loop Time (ms)", robot.loopTimeMs);
        telemetry.addData("Loop Frequency (Hz)", robot.loopHz);
        telemetry.update();
    }
}
