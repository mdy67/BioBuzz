package org.firstinspires.ftc.teamcode.Utils;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;

public class DataTransfer {
    public static double autoX = 0.0;
    public static double autoY = 0.0;
    public static double autoHeading = 0.0;
    public static double autoTurretAngle = 0.0;
    public static boolean hasRunAuto = false;

    public static void autoUpdate(Robot robot) {
        if (robot != null) {
            if (robot.drivetrain != null) {
                autoX = robot.drivetrain.x;
                autoY = robot.drivetrain.y;
                autoHeading = robot.drivetrain.head;
            }
            if (robot.turret != null) {
                autoTurretAngle = robot.turret.getTurretAngleDegrees();
            }
            hasRunAuto = true;
        }
    }

    public static void teleopInit(Robot robot) {
        if (hasRunAuto && robot != null) {
            if (robot.drivetrain != null) {
                robot.drivetrain.setPosition(autoX, autoY, autoHeading);
            }
            if (robot.turret != null) {
                robot.turret.setTurretAngleDegrees(autoTurretAngle);
            }
        }
    }
}
