package org.firstinspires.ftc.teamcode.COMPopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

@Autonomous(name = "2 TIP AUTO GUARANTEE", group = "Autonomous")
public class TwoTipGuarantee extends LinearOpMode {

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
            robot.shooter.setTargetRPM(TARGET_RPM);
            robot.update();
            idle();
        }
    }

    public double TARGET_RPM = 2900;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        robot.shooter.state = Shooter.State.SHOOTING;
        robot.turret.state = Turret.State.TRACKING;
        robot.turret.setGoal(Turret.Goal.GOAL_R);
        robot.intake.setFlowerUp();

        // Init Loop: Continuously set position and update odometry before start
        while (opModeInInit()) {
            robot.drivetrain.setPosition(65.0, -18.25, 180.0);
            robot.drivetrain.state = Drivetrain.State.IDLE;
            robot.update();

            telemetry.addData("Status", "Init Loop: Waiting for Start");
            telemetry.addData("Init X (in)", robot.drivetrain.x);
            telemetry.addData("Init Y (in)", robot.drivetrain.y);
            telemetry.addData("Init Heading (deg)", robot.drivetrain.head);
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case SHOOTING_POS_R:
                    if (!commandSent) {
                        robot.drivetrain.strafeSquidHeading(60.0, -18, 180.0, 1, 8.0, 1.5, 2.0);
                        commandSent = true;
                    } else if (robot.drivetrain.atPoint) {
                        if (splineCounter == 0) {
                            pause(2, robot);
                        } else {
                            pause(1, robot);
                        }

                        robot.shoot(1, TARGET_RPM, this);
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
                        robot.drivetrain.strafeSquidHeading(52.0, -64.0, 0.0, 1, 14.0, 3.0, 2.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 1) {
                        robot.intake.state = Intake.State.INTAKING;
                        robot.drivetrain.strafeSquidHeading(62.5, -64.0, 0.0, 1, 8.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 2) {
                        robot.drivetrain.strafeSquidHeading(58.0, -62.0, 0.0, 1, 8.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 3) {
                        robot.drivetrain.strafeSquidHeading(62.5, -64.0, 0.0, 1, 8.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 4) {

                        robot.drivetrain.strafeSquidHeading(12.0, -44.0, 340.0, 1, 14.0, 14.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 5) {
                        robot.intake.state = Intake.State.IDLE;
                        robot.drivetrain.strafeSquidHeading(-51.0, -40.0, 0.0, 1, 50.0, 35.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 6) {
                        commandSent = false;
                        state = State.SHOOTING_POS_L;
                    }
                    break;

                case SHOOTING_POS_L:
                    if (!commandSent) {
                        if (splineCounter != 0) {
                            robot.drivetrain.strafeSquidHeading(-55.0, -18, 0.0, 0.5, 8.0, 3.0, 2.0);
                        } else {
                            robot.drivetrain.strafeSquidHeading(-55.0, -18, 180.0, 0.5, 8.0, 3.0, 2.0);
                        }
                        robot.turret.setGoal(Turret.Goal.GOAL_L);

                        commandSent = true;
                    } else if (robot.drivetrain.atPoint) {
                        pause(1, robot);
                        if (splineCounter == 6) { robot.intake.setFlowerDown(); }
                        robot.shoot(0.75, TARGET_RPM, this);
                        commandSent = false;

                        if (splineCounter == 6) {
                            state = State.FLOWER_INTAKE_L;
                        } else if (splineCounter == 97) {
                            state = State.PARK;
                        } else {

                            state = State.FLOWER_INTAKE_R;
                        }
                        splineCounter = 0;

                    }
                    break;

                case FLOWER_INTAKE_L:
                    if (splineCounter == 0) {
                        robot.intake.setFlowerDown();
                        robot.drivetrain.strafeSquidHeading(-46.0, -22, 180.0, 0.6, 8.0, 2.0, 2.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 1) {
                        robot.intake.state = Intake.State.INTAKING;
                        robot.drivetrain.strafeSquidHeading(-54.0, -22, 180.0, 0.8, 8.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 2) {
                        pause(2, robot);
                        splineCounter ++;
                    }  else if (robot.drivetrain.atPoint && splineCounter == 3) {
                        robot.intake.state = Intake.State.IDLE;
                        robot.drivetrain.strafeSquidHeading(-49.0, -23, 180.0, 0.6, 8.0, 2.0, 4.0);
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
                        robot.drivetrain.strafeSquidHeading(-36.0, -40.0, 180.0, 1, 14.0, 10.0, 20.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 1) {
                        robot.intake.setFlowerDown();
                        robot.drivetrain.strafeSquidHeading(0.0, -41.0, 180.0, 1, 24.0, 18.0, 8.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 2) {
                        robot.intake.setFlowerDown();
                        robot.drivetrain.strafeSquidHeading(22.5, -41.0, 270.0, 0.6, 18.0, 2.0, 2.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 3) {
                        robot.intake.state = Intake.State.INTAKING;
                        robot.drivetrain.strafeSquidHeading(22.5, -54.0, 270.0, 0.6, 8.0, 2.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 4) {
                        pause(2, robot);
                        splineCounter ++;
                    }  else if (robot.drivetrain.atPoint && splineCounter == 5) {
                        robot.intake.state = Intake.State.IDLE;
                        robot.drivetrain.strafeSquidHeading(-30.0, -35.0, 0.0, 1, 8.0, 8.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 6) {
                        robot.intake.setFlowerUp();
                        commandSent = false;
                        splineCounter = 97;
                        state = State.SHOOTING_POS_L;
                    }
                    break;

                case PARK:
                    robot.intake.state = Intake.State.IDLE;
                    robot.shooter.state = Shooter.State.IDLE;
                    robot.turret.state = Turret.State.IDLE;
                    if (splineCounter == 0) {
                        robot.drivetrain.strafeSquidHeading(-36.0, -40.0, 90.0, 1, 14.0, 10.0, 4.0);
                        splineCounter ++;
                    } else if (robot.drivetrain.atPoint && splineCounter == 1) {
                        robot.drivetrain.strafeSquidHeading(-20.0, -58.0, 90.0, 1, 18.0, 10.0, 4.0);
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
                    robot.shooter.state = Shooter.State.IDLE;
                    robot.turret.state = Turret.State.IDLE;
                    break;
            }
            // LOOP
            robot.turret.updateTracking(robot.drivetrain.robotPose);

            robot.shooter.setTargetRPM(2805); // modify one in robot class too :(
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
