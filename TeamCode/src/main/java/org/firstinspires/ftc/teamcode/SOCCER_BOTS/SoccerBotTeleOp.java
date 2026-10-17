package org.firstinspires.ftc.teamcode.SOCCER_BOTS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Soccer Bot TeleOp", group = "Soccer Bots")
public class SoccerBotTeleOp extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private static final double SPEED = 0.5;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse one motor if needed so both wheels drive forward together.
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Soccer Bot Ready");
        telemetry.addLine("Left joystick: Forward / Backward");
        telemetry.addLine("Right joystick: Turn");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Forward/backward from left joystick
            double drive = gamepad1.left_stick_y;

            // Turning from right joystick
            double turn = -gamepad1.right_stick_x;

            // Combine driving and turning
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Normalize to preserve the turning ratio
            // while allowing turning at full driving input.
            double maxPower = Math.max(
                    1.0,
                    Math.max(Math.abs(leftPower), Math.abs(rightPower))
            );

            leftPower = (leftPower / maxPower) * SPEED;
            rightPower = (rightPower / maxPower) * SPEED;

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            telemetry.addData("Left Power", "%.2f", leftPower);
            telemetry.addData("Right Power", "%.2f", rightPower);
            telemetry.update();
        }
    }
}