package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop() {
        // 1. Drivetrain control
        double strafe = gamepad1.left_stick_x;
        double forward = -gamepad1.left_stick_y; // Stick up is negative in FTC
        double turn = gamepad1.right_stick_x;

        robot.drivetrain.driveTeleOp(strafe, forward, turn);

        // 2. Shooter & Intake controls
        if (gamepad1.right_trigger > 0.1) {
            robot.shooter.state = Shooter.State.SHOOTING;
            robot.intake.state = Intake.State.TRANSFERRING;
        } else if (gamepad1.left_bumper) {
            robot.shooter.state = Shooter.State.IDLE;
            robot.intake.state = Intake.State.INTAKING;
        } else if (gamepad1.left_trigger > 0.1) {
            robot.shooter.state = Shooter.State.IDLE;
            robot.intake.state = Intake.State.OUTTAKING;
        } else {
            robot.shooter.state = Shooter.State.IDLE;
            robot.intake.state = Intake.State.IDLE;
        }

        // 3. Turret control
        if (robot.turret.state == Turret.State.SOTM) {
            robot.turret.updateSOTM(
                    robot.drivetrain.robotPose,
                    robot.drivetrain.XVel, robot.drivetrain.YVel, robot.drivetrain.TVel,
                    strafe, forward, turn
            );
        } else if (robot.turret.state == Turret.State.TRACKING) {
            robot.turret.updateTracking(robot.drivetrain.robotPose);
        }

        // 4. Periodic update of all subsystems
        robot.update();

        // Telemetry
        telemetry.addData("Shooter State", robot.shooter.state);
        telemetry.addData("Intake State", robot.intake.state);
        telemetry.addData("Turret State", robot.turret.state);
        telemetry.addData("Robot Heading (deg)", robot.drivetrain.head);
        telemetry.update();
    }
}
