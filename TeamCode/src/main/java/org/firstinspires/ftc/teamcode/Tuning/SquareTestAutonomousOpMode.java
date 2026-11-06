package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Autonomous(name = "30501 AUTO", group = "Autonomous")
public class SquareTestAutonomousOpMode extends LinearOpMode {

    public enum State {
        SHOOTING_POS_R,
        CORNER_INTAKES,
        SHOOTING_POS_L,
        FLOWER_INTAKE_L,
        FLOWER_INTAKE_R,
        PARK,
        DONE
    }
// starting state
    private State state = State.SHOOTING_POS_R;
    private boolean commandSent = false;
    private int splineCounter = 0;

    /**
     * Helper function to pause for a set amount of seconds while keeping robot subsystems and odometry updating.
     */
    public void pause(double seconds, Robot robot) {
        ElapsedTime pauseTimer = new ElapsedTime();
        while (opModeIsActive() && pauseTimer.seconds() < seconds) {
            robot.turret.updateTracking(robot.drivetrain.robotPose);
            robot.shooter.setTargetRPM(2805);
            robot.update();
            idle();
        }
    }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        // Initialize Odometry position at origin (0, 0, 0)
        robot.drivetrain.setPosition(65.0, -18.25, 180.0);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        robot.shooter.state = Shooter.State.SHOOTING;
        robot.turret.state = Turret.State.TRACKING;
        robot.turret.setGoal(Turret.Goal.GOAL_R);
        robot.intake.setFlowerUp();
        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case SHOOTING_POS_R:
                    if (!commandSent) {
                        robot.drivetrain.strafeSquidHeading(51.0, -18, 180.0, 0.4, 8.0, 3.0, 2.0);
                        commandSent = true;
                    } else if (robot.drivetrain.atPoint) {
                        pause(2, robot);
                        robot.shoot(1, this);
                        commandSent = false;
                        robot.turret.setGoal(Turret.Goal.GOAL_L);
                        if (splineCounter == 0) {
                            state = State.CORNER_INTAKES;
                        } else {
                            splineCounter = 0;
                            state = State.PARK;
                        }



                    }
                    break;

                case CORNER_INTAKES:
                    if (splineCounter == 0) {
                        robot.drivetrain.strafeSquidHeading(52.0, -60.0, 0.0, 1, 14.0, 3.0, 2.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 1) {
                        robot.intake.state = Intake.State.INTAKING;
                        robot.drivetrain.strafeSquidHeading(60.0, -60.0, 0.0, 0.4, 8.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 2) {
                        robot.drivetrain.strafeSquidHeading(50.0, -60.0, 0.0, 1, 8.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 3) {
                        robot.drivetrain.strafeSquidHeading(60.0, -60.0, 0.0, 0.4, 8.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 4) {
                        robot.intake.state = Intake.State.IDLE;
                        robot.drivetrain.strafeSquidHeading(12.0, -44.0, 320.0, 1, 14.0, 14.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 5) {
                        robot.drivetrain.strafeSquidHeading(-51.0, -40.0, 0.0, 1, 14.0, 10.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 6) {
                        commandSent = false;
                        state = State.SHOOTING_POS_L;
                    }
                    break;

                case SHOOTING_POS_L:
                    if (!commandSent) {
                        robot.turret.setGoal(Turret.Goal.GOAL_L);
                        robot.drivetrain.strafeSquidHeading(-51.0, -18, 180.0, 0.4, 8.0, 3.0, 2.0);
                        commandSent = true;
                    } else if (robot.drivetrain.atPoint) {
                        pause(2, robot);
                        robot.shoot(1, this);
                        commandSent = false;

                        if (splineCounter == 6) {
                            state = State.FLOWER_INTAKE_L;
                        } else {
                            state = State.FLOWER_INTAKE_R;
                        }
                        splineCounter = 0;

                    }
                    break;

                case FLOWER_INTAKE_L:
                    if (splineCounter == 0) {
                        robot.intake.setFlowerDown();
                        robot.drivetrain.strafeSquidHeading(-54.0, -20.0, 180.0, 0.6, 8.0, 2.0, 2.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 1) {
                        robot.intake.state = Intake.State.INTAKING;
                        robot.drivetrain.strafeSquidHeading(-59.0, -20.0, 180.0, 0.6, 8.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 2) {
                        pause(3, robot);
                        splineCounter ++;
                    }  else if (robot.drivetrain.atPoint && splineCounter == 3) {
                        robot.intake.state = Intake.State.IDLE;
                        robot.drivetrain.strafeSquidHeading(-50.0, -20.0, 180.0, 0.6, 8.0, 3.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 4) {
                        robot.intake.setFlowerUp();
                        commandSent = false;
                        splineCounter = 0;
                        state = State.SHOOTING_POS_L;
                    }
                    break;

                case FLOWER_INTAKE_R:
                    if (splineCounter == 0) {
                        robot.drivetrain.strafeSquidHeading(-36.0, -40.0, 0.0, 1, 14.0, 10.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 1) {
                        robot.intake.setFlowerDown();
                        robot.drivetrain.strafeSquidHeading(20.0, -46.0, 270.0, 1, 18.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 2) {
                        robot.intake.state = Intake.State.INTAKING;
                        robot.drivetrain.strafeSquidHeading(20.0, -54.0, 270.0, 0.6, 8.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 3) {
                        pause(3, robot);
                        splineCounter ++;
                    }  else if (robot.drivetrain.atPoint && splineCounter == 4) {
                        robot.intake.state = Intake.State.IDLE;
                        robot.drivetrain.strafeSquidHeading(48.0, -35.0, 180.0, 1, 8.0, 8.0, 4.0);
                        splineCounter ++;
                    } else if (splineCounter == 4) {
                        robot.intake.setFlowerUp();
                        commandSent = false;
                        splineCounter = 99;
                        state = State.SHOOTING_POS_R;
                    }
                    break;

                case PARK:
                    robot.intake.state = Intake.State.IDLE;
                    robot.shooter.state = Shooter.State.IDLE;
                    robot.turret.state = Turret.State.IDLE;
                    if (splineCounter == 0) {
                        robot.drivetrain.strafeSquidHeading(36.0, -40.0, 180.0, 1, 14.0, 10.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 1) {
                        robot.drivetrain.strafeSquidHeading(0.0, -46.0, 180.0, 1, 18.0, 10.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 2) {
                        robot.drivetrain.strafeSquidHeading(-21.0, -57.0, 90.0, 0.4, 8.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 3) {
                        robot.intake.setFlowerUp();
                        commandSent = false;
                        splineCounter = 99;
                        state = State.DONE;
                    }
                    break;

                case DONE:
                default:
                    robot.drivetrain.setMotorPowers(0, 0, 0, 0);
                    robot.drivetrain.state = Drivetrain.State.IDLE;
                    break;
            }
            // LOOP
            robot.turret.updateTracking(robot.drivetrain.robotPose);
            robot.shooter.setTargetRPM(2805);
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
