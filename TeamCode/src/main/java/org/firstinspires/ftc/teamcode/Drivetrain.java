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
        FOLLOW_CURVE,
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

    // Curve Path parameters
    public double curveStartX, curveStartY;
    public double curveEndX, curveEndY;
    public double curveOffset;
    public double curveEndHeading;
    public double lookaheadDistance = 8.0; // 8 inches lookahead along curve

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

    public void followCurve(double startX, double startY, double endX, double endY, double curveOffset, double endHeading, double power, double decelThreshold, double xyThreshold, double hThreshold) {
        this.state = State.FOLLOW_CURVE;
        this.atPoint = false;

        this.curveStartX = startX;
        this.curveStartY = startY;
        this.curveEndX = endX;
        this.curveEndY = endY;
        this.curveOffset = curveOffset;
        this.curveEndHeading = endHeading;

        this.targetX = endX;
        this.targetY = endY;
        this.targetT = endHeading;

        this.powerScale = power;
        this.decelThreshold = decelThreshold;
        this.xyThreshold = xyThreshold;
        this.hThreshold = hThreshold;
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
        double distanceToTarget = Math.hypot(globxErr, globyErr);

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

    public void updateFollowCurve() {
        double distToEnd = Math.hypot(curveEndX - x, curveEndY - y);

        // If close to endpoint, switch to PID deceleration for exact arrival
        if (distToEnd <= decelThreshold) {
            targetX = curveEndX;
            targetY = curveEndY;
            targetT = curveEndHeading;
            updateGoToPoint();
            return;
        }

        // Calculate Quadratic Bezier control point perpendicular to start-end segment
        double midX = (curveStartX + curveEndX) / 2.0;
        double midY = (curveStartY + curveEndY) / 2.0;

        double dx = curveEndX - curveStartX;
        double dy = curveEndY - curveStartY;
        double len = Math.hypot(dx, dy);

        double normX = (len > 1e-4) ? -dy / len : 0;
        double normY = (len > 1e-4) ? dx / len : 0;

        double ctrlX = midX + normX * curveOffset;
        double ctrlY = midY + normY * curveOffset;

        // Estimate current parameter t on curve (closest point)
        double bestT = 0.0;
        double minDistSq = Double.MAX_VALUE;
        for (int i = 0; i <= 20; i++) {
            double t = i / 20.0;
            double bx = (1 - t) * (1 - t) * curveStartX + 2 * (1 - t) * t * ctrlX + t * t * curveEndX;
            double by = (1 - t) * (1 - t) * curveStartY + 2 * (1 - t) * t * ctrlY + t * t * curveEndY;
            double distSq = (bx - x) * (bx - x) + (by - y) * (by - y);
            if (distSq < minDistSq) {
                minDistSq = distSq;
                bestT = t;
            }
        }

        // Advance parameter t by lookahead distance (~8 inches along curve)
        double lookaheadT = Math.min(1.0, bestT + (lookaheadDistance / Math.max(1.0, len)));

        // Lookahead point on Bezier curve
        double lookX = (1 - lookaheadT) * (1 - lookaheadT) * curveStartX + 2 * (1 - lookaheadT) * lookaheadT * ctrlX + lookaheadT * lookaheadT * curveEndX;
        double lookY = (1 - lookaheadT) * (1 - lookaheadT) * curveStartY + 2 * (1 - lookaheadT) * lookaheadT * ctrlY + lookaheadT * lookaheadT * curveEndY;

        // Tangent heading along curve for smooth body orientation
        double tangX = 2 * (1 - lookaheadT) * (ctrlX - curveStartX) + 2 * lookaheadT * (curveEndX - ctrlX);
        double tangY = 2 * (1 - lookaheadT) * (ctrlY - curveStartY) + 2 * lookaheadT * (curveEndY - ctrlY);
        double targetHeading = Math.toDegrees(Math.atan2(tangY, tangX));

        // Drive toward lookahead point at max power
        double globxErr = lookX - x;
        double globyErr = lookY - y;

        double headRad = Math.toRadians(head);
        double strafeErr = globxErr * Math.cos(headRad) + globyErr * Math.sin(headRad);
        double forwardErr = globyErr * Math.cos(headRad) - globxErr * Math.sin(headRad);

        double translationMag = Math.hypot(strafeErr, forwardErr);
        double strafePower = (translationMag > 1e-4) ? (strafeErr / translationMag) * powerScale : 0;
        double forwardPower = (translationMag > 1e-4) ? (forwardErr / translationMag) * powerScale : 0;

        double headingErr = wrapAngleDegrees(targetHeading - head);
        double turnPower = tSquid.SquidOutput(headingErr);

        setWeightedMotorPowers(strafePower, forwardPower, turnPower);
    }

    public void update() {
        updatePosData(currentPinpoint);

        switch (state) {
            case GO_TO_POINT:
            case HOLD_POINT:
                updateGoToPoint();
                break;

            case FOLLOW_CURVE:
                updateFollowCurve();
                break;

            case TELEOP:
            case IDLE:
            default:
                break;
        }
    }
}
