package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TEST FLYWHEEL OPMODE", group = "Test")
public class TEST_FLYWHEEL_OPMODE extends OpMode {

    private DcMotor motor, motor1, motor3;
    private Servo servo;

    private double power = 0.0;
    private double servoPos = 0.5;
    private final double step = 0.02;

    private double motor3Power = 0.0;
    private final double motor3Step = 0.0002;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {

        double input = -gamepad1.right_stick_y; // up is positive

        if (Math.abs(input) > 0.05) {
            power += input * 0.001;
        }

        double servinput = -gamepad1.left_stick_y; // up is positive

        if (Math.abs(servinput) > 0.05) {
            servoPos += servinput * 0.0001;
        }

        // Clip values
        servoPos = Math.max(0.21, Math.min(0.56, servoPos));
        power = Math.max(-1.0, Math.min(1.0, power));

        // ===== motor3 control =====
        if (gamepad1.dpad_up) {
            motor3Power += motor3Step;
        }
        if (gamepad1.dpad_down) {
            motor3Power -= motor3Step;
        }

        motor3Power = Math.max(-1.0, Math.min(1.0, motor3Power));

        if (gamepad1.right_bumper) {
            motor3.setPower(motor3Power);
        } else {
            motor3.setPower(0);
        }
        // ==========================

        servo.setPosition(servoPos);

        motor.setPower(power);
        motor1.setPower(-power);

        telemetry.addData("Motor Power", power);
        telemetry.addData("Hood Pos", servo.getPosition());
        telemetry.addData("Motor3 Set Power", motor3Power);
        telemetry.addData("Motor3 Applied", gamepad1.right_bumper ? motor3Power : 0);
        telemetry.update();
    }
}