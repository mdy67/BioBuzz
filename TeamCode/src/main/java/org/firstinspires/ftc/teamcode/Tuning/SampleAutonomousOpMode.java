package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Config
@Autonomous(name = "Sample Autonomous", group = "Autonomous")
public class SampleAutonomousOpMode extends LinearOpMode {

    private Robot robot;

    // Configurable Flywheel Target RPM at the top for all autonomous shooting areas
    public static double autoTargetRPM = 2805.0;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        // 1. Initialize Odometry position
        robot.drivetrain.setPosition(65.25, -18, 180.0);

        // 2. Select Goal R as active goal
        robot.turret.setGoal(Turret.Goal.GOAL_R);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // === STEP 1: Spin up flywheel and drive to first point ===
            robot.turret.state = Turret.State.TRACKING;
            robot.shooter.state = Shooter.State.SHOOTING;
            robot.shooter.setTargetRPM(autoTargetRPM);

            robot.driveTo(56.0, -24.0, 270.0, 0.5, this);

            // === STEP 2: Wait until flywheel is within ~70 RPM of target ===
            while (opModeIsActive() && Math.abs(robot.shooter.getCurrentRPM() - robot.shooter.targetRPM) > 70.0) {
                robot.update();
                idle();
            }

            // === STEP 3: Shoot ===
            robot.shoot(1.0, autoTargetRPM, this);

            // === STEP 4: Drive to next point while intaking ===
            robot.intake.state = Intake.State.INTAKING;
            robot.drivetrain.strafeSquidHeading(24.0, 48.0, 90.0, 1.0, 8.0, 1.0, 1.0);
            while (opModeIsActive() && !robot.drivetrain.atPoint) {
                robot.update();
                idle();
            }
            robot.intake.state = Intake.State.IDLE;

            // === STEP 5: Switch goal to Goal L and idle subsystems ===
            robot.turret.setGoal(Turret.Goal.GOAL_L);
            robot.turret.state = Turret.State.IDLE;
            robot.shooter.state = Shooter.State.IDLE;
            robot.intake.state = Intake.State.IDLE;

            while (opModeIsActive()) {
                robot.update();
                telemetry.addData("Status", "Autonomous Complete");
                telemetry.update();
            }
        }
    }
}
