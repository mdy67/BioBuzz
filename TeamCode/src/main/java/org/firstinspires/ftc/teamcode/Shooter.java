package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    public enum State {
        IDLE,
        TESTING,
        SHOOTING
    }

    public State state = State.IDLE;

    public DcMotorEx flywheelL;
    public DcMotorEx flywheelR;

    // Open-loop target power (0.0 to 1.0)
    public double targetPower = 0.0;

    // Compatibility stubs for tuning opmode
    public static double kP = 0.0006;
    public static double kI = 0.00008;
    public static double kD = 0.0001;
    public static double kF = 0.00017;
    public static double MIN_POWER = 0.08;
    public double targetRPM = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        flywheelL = hardwareMap.get(DcMotorEx.class, "flywheelL");
        flywheelR = hardwareMap.get(DcMotorEx.class, "flywheelR");

        flywheelL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelL.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelR.setDirection(DcMotorSimple.Direction.FORWARD); // Spin opposite each other
    }

    public void setPower(double power) {
        this.targetPower = Math.max(0.0, Math.min(1.0, power));
    }

    public void adjustPower(double delta) {
        setPower(this.targetPower + delta);
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    public double getCurrentRPM() {
        return 0.0;
    }

    public double getBatteryVoltage() {
        return 12.0;
    }

    public double calculateVelocityPower() {
        return targetPower;
    }

    public void update() {
        switch (state) {
            case TESTING:
            case SHOOTING:
                flywheelL.setPower(targetPower);
                flywheelR.setPower(targetPower); // Spin opposite each other
                break;

            case IDLE:
            default:
                flywheelL.setPower(0.0);
                flywheelR.setPower(0.0);
                break;
        }
    }
}
