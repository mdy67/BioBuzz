package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Sample Autonomous", group = "Autonomous")
public class SampleAutonomousOpMode extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        // 1. Initialize Odometry position
        robot.drivetrain.setPosition(65.25, -18, 90.0);

        // 2. Select Goal R as active goal
        robot.turret.setGoal(Turret.Goal.GOAL_R);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // === STEP 1: Drive to (0, 24) with SOTM turret mode enabled ===
            robot.turret.state = Turret.State.SOTM;
            robot.driveTo(0.0, 24.0, 0.0, 1.0, this);

            // === STEP 2: Drive to (24, 48) with tracking turret mode enabled ===
            robot.turret.state = Turret.State.TRACKING;
            robot.driveTo(24.0, 48.0, 90.0, 1.0, this);

            // === STEP 3: Lock turret and shoot for 3 seconds (runs intake/transfer and quintic RPM automatically) ===
            robot.turret.state = Turret.State.LOCKED;
            robot.turret.setLockAngle(90.0);
            robot.shoot(3.0, this);

            // === STEP 4: Switch goal to Goal L and idle subsystems ===
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
