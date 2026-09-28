package org.firstinspires.ftc.teamcode.Utils;

public class DTSQUID {
    public double kP;
    public double kD;

    public DTSQUID (double P, double D) {
        kP = P;
        kD = D;
    }

    double lastError = 0;
    int counter = 0;

    double lastLoopTime = System.nanoTime();
    double loopTime = 0.0;
    double currentTime = System.nanoTime();

    double proportion, derivative;
    double output = 0.0;

    public double SquidOutput(double error) {
        if (counter == 0) {
            lastLoopTime = System.nanoTime() - 10_000_000;
        }

        currentTime = System.nanoTime();
        loopTime = (currentTime - lastLoopTime) / 1_000_000_000.0;
        lastLoopTime = currentTime;

        proportion = kP * error;
        derivative = kD * (error - lastError) / loopTime;

        lastError = error;
        counter++;

        output = proportion + derivative;
        return Math.sqrt(Math.abs(output) * Math.signum(output));
    }

}
