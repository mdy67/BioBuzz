package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Turret Auto-Tuner", group = "Test")
public class TurretAutoTunerOpMode extends OpMode {

    public enum TuningPhase {
        IDLE,
        TUNING_TURRET,
        COMPLETED
    }

    private TuningPhase phase = TuningPhase.IDLE;
    private Robot robot;
    private final ElapsedTime cycleTimer = new ElapsedTime();

    // Tunable gains being optimized iteratively
    public static double opt_kP = 0.0007;
    public static double opt_kI = 0.0008;
    public static double opt_kD = 0.00008;
    public static double opt_kS = 0.05;

    // Test cycle state variables
    private double testTargetAngle = 45.0;
    private double cycleStartTime = 0.0;
    private double peakOvershoot = 0.0;
    private double finalError = 0.0;
    private double timeToSettle = 0.0;
    private int crossingCount = 0;
    private boolean hasCrossed = false;
    private boolean settled = false;
    private int iterationCount = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.turret.state = Turret.State.LOCKED;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Start auto-tuning sequence when pressing the A button
        if (gamepad1.a && phase == TuningPhase.IDLE) {
            phase = TuningPhase.TUNING_TURRET;
            cycleTimer.reset();
            startNewCycle(45.0);
        }

        switch (phase) {
            case TUNING_TURRET:
                runTurretEpoch();
                break;
            case COMPLETED:
                robot.turret.state = Turret.State.IDLE;
                break;
            case IDLE:
            default:
                robot.turret.state = Turret.State.LOCKED;
                robot.turret.targetAngle = 0.0;
                break;
        }

        robot.update();

        // Telemetry status
        double currentAngle = robot.turret.getTurretAngleDegrees();
        double constrainedTarget = robot.turret.getClosestValidTarget(robot.turret.targetAngle, currentAngle);
        double error = constrainedTarget - currentAngle;

        telemetry.addData("Auto-Tuner Phase", phase);
        telemetry.addData("Iteration", iterationCount);
        telemetry.addData("Target Angle", robot.turret.targetAngle);
        telemetry.addData("Current Angle", currentAngle);
        telemetry.addData("Error", error);
        telemetry.addData("Peak Overshoot", peakOvershoot);
        telemetry.addData("Time to Settle (s)", timeToSettle);
        telemetry.addData("Current kP", opt_kP);
        telemetry.addData("Current kD", opt_kD);
        telemetry.addData("Press A to Start Auto-Tune", "(Gamepad1 A)");
        telemetry.update();
    }

    private void startNewCycle(double target) {
        testTargetAngle = target;
        robot.turret.targetAngle = testTargetAngle;
        cycleStartTime = cycleTimer.seconds();
        peakOvershoot = 0.0;
        crossingCount = 0;
        hasCrossed = false;
        settled = false;
        timeToSettle = 0.0;
    }

    private void runTurretEpoch() {
        Turret.kP = opt_kP;
        Turret.kI = opt_kI;
        Turret.kD = opt_kD;
        Turret.kS = opt_kS;
        robot.turret.state = Turret.State.LOCKED;

        double currentAngle = robot.turret.getTurretAngleDegrees();
        double constrainedTarget = robot.turret.getClosestValidTarget(testTargetAngle, currentAngle);
        double error = constrainedTarget - currentAngle;
        double elapsed = cycleTimer.seconds() - cycleStartTime;

        if (Math.abs(error) > 1.0) {
            double prevError = constrainedTarget - robot.turret.getTurretAngleDegrees();
            if (!settled && Math.signum(error) != Math.signum(prevError)) {
                if (!hasCrossed) {
                    hasCrossed = true;
                    crossingCount++;
                }
            }
            if (Math.abs(error) > peakOvershoot && elapsed > 0.2) {
                peakOvershoot = Math.abs(error);
            }
        } else {
            if (!settled) {
                settled = true;
                timeToSettle = elapsed;
            }
        }

        if (elapsed > 3.0) {
            finalError = Math.abs(error);
            iterationCount++;

            if (peakOvershoot > 1.5 || crossingCount > 1) {
                opt_kD += 0.00001;
                opt_kP *= 0.95;
            } else if (timeToSettle > 2.0 || finalError > 1.0) {
                opt_kP *= 1.05;
            } else {
                phase = TuningPhase.COMPLETED;
                return;
            }

            startNewCycle(testTargetAngle == 45.0 ? 0.0 : 45.0);
        }
    }
}
