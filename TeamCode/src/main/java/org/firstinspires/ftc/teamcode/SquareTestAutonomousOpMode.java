package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Square Test Autonomous", group = "Autonomous")
public class SquareTestAutonomousOpMode extends LinearOpMode {

    public enum State {
        DRIVE_1,
        DRIVE_2,
        DRIVE_3,
        DRIVE_4,
        DONE
    }

    private State state = State.DRIVE_1;
    private boolean commandSent = false;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        // Initialize Odometry position at origin (0, 0, 0)
        robot.drivetrain.setPosition(0.0, 0.0, 0.0);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case DRIVE_1:
                    if (!commandSent) {
                        robot.drivetrain.strafeSquidHeading(0.0, 36.0, 90.0, 1, 14.0, 3.0, 2.0);
                        commandSent = true;
                    } else if (robot.drivetrain.atPoint) {
                        commandSent = false;
                        state = State.DRIVE_2;
                    }
                    break;

                case DRIVE_2:
                    if (!commandSent) {
                        robot.drivetrain.strafeSquidHeading(36.0, 36.0, 180.0, 1, 14.0, 3.0, 2.0);
                        commandSent = true;
                    } else if (robot.drivetrain.atPoint) {
                        commandSent = false;
                        state = State.DRIVE_3;
                    }
                    break;

                case DRIVE_3:
                    if (!commandSent) {
                        robot.drivetrain.strafeSquidHeading(36.0, 0.0, 270.0, 1, 14.0, 3.0, 2.0);
                        commandSent = true;
                    } else if (robot.drivetrain.atPoint) {
                        commandSent = false;
                        state = State.DRIVE_4;
                    }
                    break;

                case DRIVE_4:
                    if (!commandSent) {
                        robot.drivetrain.strafeSquidHeading(0.0, 0.0, 0.0, 1, 14.0, 3.0, 2.0);
                        commandSent = true;
                    } else if (robot.drivetrain.atPoint) {
                        commandSent = false;
                        state = State.DONE;
                    }
                    break;

                case DONE:
                default:
                    robot.drivetrain.setMotorPowers(0, 0, 0, 0);
                    robot.drivetrain.state = Drivetrain.State.IDLE;
                    break;
            }

            robot.update();

            // Detailed telemetry for debugging odometry, pathing, and target heading
            telemetry.addData("Square FSM State", state);
            telemetry.addData("Current X (in)", robot.drivetrain.x);
            telemetry.addData("Current Y (in)", robot.drivetrain.y);
            telemetry.addData("Current Heading (deg)", robot.drivetrain.head);
            telemetry.addData("Target X (in)", robot.drivetrain.targetX);
            telemetry.addData("Target Y (in)", robot.drivetrain.targetY);
            telemetry.addData("Target Heading (deg)", Math.toDegrees(robot.drivetrain.targetT));
            telemetry.addData("Global X Error (in)", robot.drivetrain.globalXerror);
            telemetry.addData("Global Y Error (in)", robot.drivetrain.globalYerror);
            telemetry.addData("Heading Error (deg)", Math.toDegrees(robot.drivetrain.tError));
            telemetry.addData("At Point?", robot.drivetrain.atPoint);
            telemetry.addData("LF Power", robot.drivetrain.leftFront.getPower());
            telemetry.addData("LB Power", robot.drivetrain.leftBack.getPower());
            telemetry.addData("RB Power", robot.drivetrain.rightBack.getPower());
            telemetry.addData("RF Power", robot.drivetrain.rightFront.getPower());
            telemetry.addData("Loop Frequency (Hz)", robot.loopHz);
            telemetry.update();
        }
    }
}
