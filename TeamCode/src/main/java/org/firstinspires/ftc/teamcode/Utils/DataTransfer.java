package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Robot;

public class DataTransfer {
    public static double autoX = 0.0;
    public static double autoY = 0.0;
    public static double autoHeading = 0.0;
    public static int autoEncoderOffset = 0;
    public static boolean hasRunAuto = false;

    public static void autoUpdate(Robot robot) {
        if (robot != null) {
            if (robot.drivetrain != null) {
                autoX = robot.drivetrain.x;
                autoY = robot.drivetrain.y;
                autoHeading = Drivetrain.wrapAngle360(robot.drivetrain.head);
            }
            if (robot.turret != null) {
                autoEncoderOffset = robot.turret.getEncoderOffset();
            }
            hasRunAuto = true;
        }
    }

    public static void teleopInit(Robot robot) {
        if (hasRunAuto && robot != null) {
            if (robot.drivetrain != null) {
                robot.drivetrain.setPosition(autoX, autoY, autoHeading+180);
            }
            if (robot.turret != null) {
                robot.turret.setEncoderOffset(autoEncoderOffset);
            }
        }
    }
}
