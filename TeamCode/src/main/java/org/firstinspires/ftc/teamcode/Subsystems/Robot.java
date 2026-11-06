package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {

    public Drivetrain drivetrain;
    public Shooter shooter;
    public Intake intake;
    public Turret turret;

    private long lastLoopTime = System.nanoTime();
    public double loopTimeMs = 0.0;
    public double loopHz = 0.0;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    /**
     * Clean one-line action to drive to a point while continuously updating subsystems, odometry, and turret aiming.
     */
    public void driveTo(double x, double y, double heading, double power, LinearOpMode opMode) {
        drivetrain.strafeSquidHeading(x, y, heading, power, 8.0, 1.0, 1.0);
        while (opMode.opModeIsActive() && !drivetrain.atPoint) {
            update();
            opMode.idle();
        }
    }

    /**
     * Clean one-line action to run intake/transfer and shoot for a specified duration while updating turret/quintic RPM and configurable target RPM.
     */
    public void shoot(double durationSeconds, double targetRPM, LinearOpMode opMode) {
        ElapsedTime shootTimer = new ElapsedTime();
        shooter.state = Shooter.State.SHOOTING;
        intake.rampScale = 0.0;
        shooter.powerBoost = 0.0;
        while (opMode.opModeIsActive() && shootTimer.seconds() < durationSeconds) {
            shooter.setTargetRPM(targetRPM);
            shooter.powerBoost = intake.rampScale * 0.5; // Balanced power boost (0.1 max)
            turret.setGateTransferring(true);
            intake.state = Intake.State.TRANSFERRING;
            update();
            opMode.idle();
        }
        turret.setGateTransferring(false);
        intake.state = Intake.State.IDLE;
        intake.rampScale = 0.0;
        shooter.powerBoost = 0.0;
    }

    public void shoot(double durationSeconds, LinearOpMode opMode) {
        shoot(durationSeconds, (shooter.targetRPM > 0 ? shooter.targetRPM : 3375.0 - 570), opMode);
    }

    public void update() {
        long now = System.nanoTime();
        long deltaNs = now - lastLoopTime;
        loopTimeMs = deltaNs / 1_000_000.0;
        if (loopTimeMs > 0) {
            loopHz = 1000.0 / loopTimeMs;
        }
        lastLoopTime = now;

        // 1. Update Drivetrain odometry
        drivetrain.update();

        // 2. Automatically update turret aiming based on state and raw Pinpoint pose
        if (turret.state == Turret.State.TRACKING) {
            turret.updateTracking(drivetrain.robotPose);
        } else {
            turret.state = Turret.State.IDLE;
        }

        // 3. Update all physical subsystems
        shooter.update();
        intake.update();
        turret.update();

        // 4. Save data for TeleOp transfer
        org.firstinspires.ftc.teamcode.Utils.DataTransfer.autoUpdate(this);
    }
}
