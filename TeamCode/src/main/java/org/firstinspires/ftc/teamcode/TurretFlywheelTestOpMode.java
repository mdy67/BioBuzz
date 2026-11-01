package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret & Flywheel Test", group = "Test")
public class TurretFlywheelTestOpMode extends OpMode {

    private CRServo turretServo1, turretServo2;
    private DcMotorEx flywheelMotor;
    private DcMotorEx turretEncoder;
    private DcMotorEx flywheelRpmEncoder; // plugged into "leftFront" port

    private double flywheelPower = 0.0;
    private static final double TICKS_PER_REV = 28.0; // Flywheel encoder ticks per rev

    @Override
    public void init() {
        // 1. Turret CR Servos (2 servos spinning in the same direction)
        turretServo1 = hardwareMap.get(CRServo.class, "turretServo1");
        turretServo2 = hardwareMap.get(CRServo.class, "turretServo2");

        // 2. Flywheel motor (powered by right joystick incrementally)
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelL");

        // 3. Turret encoder plugged into port named "flywheelR"
        turretEncoder = hardwareMap.get(DcMotorEx.class, "flywheelR");

        // 4. Flywheel RPM encoder plugged into front left drivetrain motor port ("leftFront")
        flywheelRpmEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
    }

    @Override
    public void loop() {
        // --- Turret CR Servos controlled by left joystick (X axis for rotation) ---
        // Power equal to joystick's position (stops when released)
        double turretInput = -gamepad1.left_stick_x;
        if (Math.abs(turretInput) < 0.05) {
            turretInput = 0.0;
        }
        turretServo1.setPower(turretInput);
        turretServo2.setPower(turretInput); // Same direction

        // --- Flywheel motor controlled incrementally by right joystick (Y axis) ---
        double flywheelInput = -gamepad1.right_stick_y; // Up is positive
        if (Math.abs(flywheelInput) > 0.05) {
            flywheelPower += flywheelInput * 0.02; // Incremental adjustment
        }
        flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower)); // Positive power only

        flywheelMotor.setPower(flywheelPower);

        // --- Telemetry ---
        telemetry.addData("Turret Joystick Input", turretInput);
        telemetry.addData("Turret Encoder Position (on flywheelR)", turretEncoder.getCurrentPosition());
        telemetry.addData("Turret Encoder Velocity", turretEncoder.getVelocity());

        telemetry.addData("Flywheel Target Power", flywheelPower);
        double ticksPerSec = flywheelRpmEncoder.getVelocity();
        double flywheelRpm = (ticksPerSec * 60.0) / TICKS_PER_REV;
        telemetry.addData("Flywheel RPM (on leftFront)", flywheelRpm);
        telemetry.update();
    }
}
