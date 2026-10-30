package org.firstinspires.ftc.teamcode;

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
     * Clean one-line action to run intake/transfer and shoot for a specified duration while updating turret/quintic RPM.
     */
    public void shoot(double durationSeconds, LinearOpMode opMode) {
        ElapsedTime shootTimer = new ElapsedTime();
        shooter.state = Shooter.State.SHOOTING;
        while (opMode.opModeIsActive() && shootTimer.seconds() < durationSeconds) {
            double distance = turret.getDistanceToGoal(drivetrain.x, drivetrain.y);
            shooter.setTargetRPM(Shooter.calculateQuinticRPM(distance));
            turret.setGateTransferring(true);
            intake.state = Intake.State.TRANSFERRING;
            update();
            opMode.idle();
        }
        turret.setGateTransferring(false);
        intake.state = Intake.State.IDLE;
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
        drivetrain.updatePosData(drivetrain.currentPinpoint);

        // 2. Automatically update turret aiming based on state and current robot pose
        if (turret.state == Turret.State.TRACKING) {
            turret.updateTracking(drivetrain.robotPose);
        } else if (turret.state == Turret.State.SOTM) {
            turret.updateSOTM(drivetrain.robotPose, drivetrain.XVel, drivetrain.YVel, 0.0, 0.0);
        }

        // 3. Update all physical subsystems
        shooter.update();
        intake.update();
        turret.update();
    }
}
