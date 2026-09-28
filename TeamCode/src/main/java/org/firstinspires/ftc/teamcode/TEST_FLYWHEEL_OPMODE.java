package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TEST FLYWHEEL OPMODE", group = "Test")
public class TEST_FLYWHEEL_OPMODE extends OpMode {

    private DcMotor motor, motor1;
    private Servo servo;

    private double power = 0.0;
    private double servoPos = 0.5;
    private double servinput = 0.0;
    private final double step = 0.02;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
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
            servoPos += servinput * step;
        }

        servoPos = Math.max(0, Math.min(1.0, servoPos));
        power = Math.max(-1.0, Math.min(1.0, power));

        servo.setPosition(servoPos);


        motor.setPower(power);
        motor1.setPower(-power);

        telemetry.addData("Motor Power", power);
        telemetry.addData("Hood Pos:", servo.getPosition());
        telemetry.update();
    }
}