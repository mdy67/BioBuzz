package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Utils.DTSQUID;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain {

    public enum State {
        GO_TO_POINT,
        TELEOP,
        HOLD_POINT,
        IDLE
    }

    public final int currentPinpoint = 1;
    public State state = State.IDLE;
    public DcMotor leftFront, leftBack, rightBack, rightFront;
    public DcMotor[] driveMotors;
    GoBildaPinpointDriver pinpoint1, pinpoint2;
    public final List<DcMotor> driveMotorList = new ArrayList<>();

    private DcMotorEx initMotor(HardwareMap hw, String name, boolean reverse) {
        DcMotorEx motor = hw.get(DcMotorEx.class, name);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(reverse
                ? DcMotorSimple.Direction.REVERSE
                : DcMotorSimple.Direction.FORWARD);

        driveMotorList.add(motor);
        return motor;
    }

    public Drivetrain(HardwareMap hardwareMap) {
        leftFront = initMotor(hardwareMap, "leftFront", false);
        leftBack = initMotor(hardwareMap, "leftBack", false);
        rightBack = initMotor(hardwareMap, "rightBack", true);
        rightFront = initMotor(hardwareMap, "rightFront", true);

        driveMotors = driveMotorList.toArray(new DcMotor[0]);

        pinpoint1 = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint1");
        pinpoint2 = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint2");

        setupPinpoint(pinpoint1);
        setupPinpoint(pinpoint2);
    }

    private void setupPinpoint(GoBildaPinpointDriver pp) {
        pp.setOffsets(69.19, -154.75, DistanceUnit.MM);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pp.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
    }

    public void setPosition(double x, double y, double heading) {
        Pose2D pose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
        pinpoint1.setPosition(pose);
        pinpoint2.setPosition(pose);
    }

    public void setMotorPowers(double lf, double lb, double rb, double rf) {
        double[] powers = {lf, lb, rb, rf};
        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i].setPower(powers[i]);
        }
    }

    public void setWeightedMotorPowers(double strafe, double forward, double turn) {
        double lf = forward - strafe - turn;
        double lb = forward + strafe - turn;
        double rb = forward - strafe + turn;
        double rf = forward + strafe + turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(lb),
                                Math.max(Math.abs(rb), Math.abs(rf)))));
        setMotorPowers(
                lf / max,
                lb / max,
                rb / max,
                rf / max
        );
    }

    public void driveTeleOp(double strafe, double forward, double turn) {
        state = State.TELEOP;
        setWeightedMotorPowers(strafe, forward, turn);
    }

    public Pose2D robotPose = new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.DEGREES, 0.0);
    public double x, y, head;
    public Pose2D lastRobotPose = robotPose;
    public Pose2D targetPose = robotPose;
    private Pose2D lastTarget = robotPose;

    public boolean atPoint = false;
    public double targetX, targetY, targetT, targetVel, xyThreshold, hThreshold;
    public double XVel, YVel, TVel;
    public double powerScale = 1.0;
    public double decelThreshold = 8.0; // Distance threshold in inches before PID deceleration kicks in

    public double xErr, yErr, tErr;
    private boolean atX, atY, atT;

    public static double akP = 0.08;
    public static double akD = 0.01;
    public static double hkP = 0.14;
    public static double hkD = 0.03;

    DTSQUID xSquid = new DTSQUID(akP, akD);
    DTSQUID ySquid = new DTSQUID(akP, akD);
    DTSQUID tSquid = new DTSQUID(hkP, hkD);

    public void updatePosData(int pinpoint) {
        if (pinpoint == 1) {
            pinpoint1.update();
            XVel = pinpoint1.getVelX(DistanceUnit.INCH);
            YVel = pinpoint1.getVelY(DistanceUnit.INCH);
            TVel = pinpoint1.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            robotPose = pinpoint1.getPosition();
        } else {
            pinpoint2.update();
            XVel = pinpoint2.getVelX(DistanceUnit.INCH);
            YVel = pinpoint2.getVelY(DistanceUnit.INCH);
            TVel = pinpoint2.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
            robotPose = pinpoint2.getPosition();
        }
        x = robotPose.getX(DistanceUnit.INCH);
        y = robotPose.getY(DistanceUnit.INCH);
        head = robotPose.getHeading(AngleUnit.DEGREES);
    }

    public static double wrapAngleDegrees(double angle) {
        double wrapped = angle;
        while (wrapped > 180.0) wrapped -= 360.0;
        while (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
    }

    public void strafeSquidHeading(double targetX, double targetY, double heading, double power, double decelThreshold, double xyThreshold, double hThreshold) {
        this.targetPose = new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.DEGREES, heading);
        this.state = State.GO_TO_POINT;
        if (targetPose != lastTarget) {
            this.atPoint = false;
            this.lastTarget = targetPose;
            this.targetX = targetX;
            this.targetY = targetY;
            this.targetT = heading;

            this.powerScale = power;
            this.decelThreshold = decelThreshold;
            this.xyThreshold = xyThreshold;
            this.hThreshold = hThreshold;
        }
    }

    public void getErrors() {
        double globxErr = targetX - x;
        double globyErr = targetY - y;
        tErr = wrapAngleDegrees(targetT - head);

        double headRad = Math.toRadians(head);
        xErr = globxErr * Math.cos(headRad) + globyErr * Math.sin(headRad);
        yErr = globyErr * Math.cos(headRad) - globxErr * Math.sin(headRad);

        atX = Math.abs(globxErr) <= xyThreshold;
        atY = Math.abs(globyErr) <= xyThreshold;
        atT = Math.abs(tErr) <= hThreshold;

        atPoint = atX && atY && atT;
    }

    public void updateGoToPoint() {
        getErrors();

        if (atPoint) {
            setMotorPowers(0, 0, 0, 0);
            state = State.HOLD_POINT;
            return;
        }

        double globxErr = targetX - x;
        double globyErr = targetY - y;
        double distanceToTarget = Math.hypot(globxErr, globyErr); // wait, Math.hypot(globxErr, globyErr)

        double strafePower = xSquid.SquidOutput(xErr);
        double forwardPower = ySquid.SquidOutput(yErr);
        double turnPower = tSquid.SquidOutput(tErr);

        // Max Power Phase: If outside decelThreshold, scale translation to max power (powerScale)
        if (distanceToTarget > decelThreshold) {
            double translationMag = Math.hypot(strafePower, forwardPower);
            if (translationMag > 1e-4) {
                strafePower = (strafePower / translationMag) * powerScale;
                forwardPower = (forwardPower / translationMag) * powerScale;
            }
        }

        setWeightedMotorPowers(strafePower, forwardPower, turnPower);
    }

    public void update() {
        updatePosData(currentPinpoint);

        switch (state) {
            case GO_TO_POINT:
            case HOLD_POINT:
                updateGoToPoint();
                break;

            case TELEOP:
            case IDLE:
            default:
                break;
        }
    }
}
