package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "Flywheel PID Tuning", group = "Test")
public class FlywheelTuningOpMode extends OpMode {

    // FTC Dashboard editable tuning parameters
    public static double TUNING_TARGET_RPM = 3000.0;
    public static double TUNING_MIN_POWER = 0.08;
    public static double TUNING_KP = 0.0006;
    public static double TUNING_KI = 0.00008;
    public static double TUNING_KD = 0.0001;
    public static double TUNING_KF = 0.00017;

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        // Send telemetry to both Driver Station and FTC Dashboard simultaneously
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Apply tuning parameters from FTC Dashboard to Shooter gains
        Shooter.MIN_POWER = TUNING_MIN_POWER;
        Shooter.kP = TUNING_KP;
        Shooter.kI = TUNING_KI;
        Shooter.kD = TUNING_KD;
        Shooter.kF = TUNING_KF;

        robot.shooter.setTargetRPM(TUNING_TARGET_RPM);
        robot.shooter.state = Shooter.State.SHOOTING;

        robot.update();

        // Telemetry monitoring for PID tuning performance
        telemetry.addData("Target RPM: ", TUNING_TARGET_RPM);
        telemetry.addData("Current RPM: ", robot.shooter.getCurrentRPM());
        telemetry.addData("RPM Error: ", TUNING_TARGET_RPM - robot.shooter.getCurrentRPM());
        telemetry.addData("Flywheel Power: ", robot.shooter.calculateVelocityPower());
        telemetry.addData("Battery Voltage (V): ", robot.shooter.getBatteryVoltage());
        telemetry.addData("Flywheel L Amps: ", robot.shooter.flywheelL.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Flywheel R Amps: ", robot.shooter.flywheelR.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Loop Time (ms): ", robot.loopTimeMs);
        telemetry.addData("Loop Frequency (Hz): ", robot.loopHz);
        telemetry.update();
    }
}
