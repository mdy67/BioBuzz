package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    public Drivetrain drivetrain;
    public Shooter shooter;
    public Intake intake;
    public Turret turret;

    private long lastLoopTime = System.nanoTime();
    public double loopTimeMs = 0.0;
    public double loopHz = 0.0;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new Drivetrain(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
    }

    public void update() {
        long now = System.nanoTime();
        long deltaNs = now - lastLoopTime;
        loopTimeMs = deltaNs / 1_000_000.0;
        if (loopTimeMs > 0) {
            loopHz = 1000.0 / loopTimeMs;
        }
        lastLoopTime = now;

        drivetrain.updatePosData(drivetrain.currentPinpoint);
        shooter.update();
        intake.update();
        turret.update();
    }
}
