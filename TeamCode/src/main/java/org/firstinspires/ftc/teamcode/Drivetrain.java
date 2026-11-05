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
import org.firstinspires.ftc.teamcode.Utils.Utils;

import java.util.ArrayList;
import java.util.List;

public class Drivetrain {

    public enum State {
        GO_TO_POINT,
        TELEOP,
        HOLD_POINT,
        IDLE
    }
    public State state = State.IDLE;

    public final int currentPinpoint = 1;
    public DcMotor leftFront, leftBack, rightBack, rightFront;
    public DcMotor[] driveMotors;
    public GoBildaPinpointDriver pinpoint;
    public final List<DcMotor> driveMotorList = new ArrayList<>();

    // TeleOp input vars
    private double teleopStrafe = 0;
    private double teleopForward = 0;
    private double teleopTurn = 0;

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

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint1");
        } catch (Exception e) {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        }

        pinpoint.setOffsets(-71.6, -96, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }

    public void setPosition(double x, double y, double heading) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading));
    }

    public void setHeading(double heading) {
        pinpoint.setHeading(heading, AngleUnit.DEGREES);
    }

    public void setMotorPowers(double lf, double lb, double rb, double rf) {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
        rightFront.setPower(rf);
    }

    public void setWeightedMotorPowers(double strafe, double fwd, double heading) {
        double denominator = Math.max(Math.abs(strafe) + Math.abs(fwd) + Math.abs(heading), 1);
        double[] weightPowers = new double[]{
                (fwd - strafe - heading) / denominator,
                (fwd + strafe - heading) / denominator,
                (fwd - strafe + heading) / denominator,
                (fwd + strafe + heading) / denominator
        };
        setMotorPowers(weightPowers[0], weightPowers[1], weightPowers[2], weightPowers[3]);
    }

    public void driveTeleOp(double strafe, double forward, double turn) {
        state = State.TELEOP;
        teleopStrafe = strafe;
        teleopForward = forward;
        teleopTurn = turn;
    }

    public Pose2D robotPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    public double x, y, head;
    public Pose2D lastrobotPose = robotPose;
    public Pose2D targetPose = new Pose2D(DistanceUnit.INCH, 0.001, 0.001, AngleUnit.DEGREES, 0.001);
    private Pose2D lastTarget = targetPose;

    public boolean atPoint = false;
    public double targetX, targetY, targetT, maxPower, xyThreshold, hThreshold;
    public double XVel, YVel, TVel;
    public double powerScale = 1.0;
    public double decelThreshold = 8.0;

    public double tError, xError, yError;
    public double globalXerror, globalYerror;
    public boolean atX, atY, atT;

    public static double akP = 0.04;
    public static double akD = 0.001;
    public static double hkP = 0.04;
    public static double hkD = 0.03;

    DTSQUID xSquid = new DTSQUID(akP, akD);
    DTSQUID ySquid = new DTSQUID(akP, akD);
    DTSQUID tSquid = new DTSQUID(hkP, hkD);

    public void goToPoint(Pose2D targetPoint, double maxPower, double xyThreshold, double hThreshold) {
        targetPose = targetPoint;
        targetX = targetPose.getX(DistanceUnit.INCH);
        targetY = targetPose.getY(DistanceUnit.INCH);
        targetT = Math.toRadians(targetPose.getHeading(AngleUnit.DEGREES));

        this.maxPower = maxPower;
        this.xyThreshold = xyThreshold;
        this.hThreshold = hThreshold;
        this.state = State.GO_TO_POINT;
    }

    public void strafeSquidHeading(double targetX, double targetY, double heading, double power, double decelThreshold, double xyThreshold, double hThreshold) {
        goToPoint(new Pose2D(DistanceUnit.INCH, targetX, targetY, AngleUnit.DEGREES, heading), power, xyThreshold, hThreshold);
        this.powerScale = power;
        this.decelThreshold = decelThreshold;
    }

    public void getErrors() {
        double heading = robotPose.getHeading(AngleUnit.RADIANS);
        tError = Utils.headingClip(targetT - heading);
        globalXerror = (targetX - x);
        globalYerror = (targetY - y);

        xError = globalXerror * Math.cos(heading) + globalYerror * Math.sin(heading);
        yError = globalYerror * Math.cos(heading) - globalXerror * Math.sin(heading);

        atX = (Math.abs(xError) <= xyThreshold);
        atY = (Math.abs(yError) <= xyThreshold);
        atT = (Math.abs(Math.toDegrees(tError)) <= hThreshold);
        atPoint = atX && atY && atT;
    }

    public boolean DTatTarget() {
        getErrors();
        return ((Math.abs(xError) + Math.abs(yError)) <= xyThreshold && Math.abs(tError) < hThreshold);
    }

    public void updateGoToPoint() {
        getErrors();

        if (atPoint) {
            setMotorPowers(0, 0, 0, 0);
            state = State.IDLE;
            return;
        }

        double globxErr = targetX - x;
        double globyErr = targetY - y;
        double distanceToTarget = Math.hypot(globxErr, globyErr);

        double strafePower = xSquid.SquidOutput(xError);
        double forwardPower = ySquid.SquidOutput(yError);
        double turnPower = tSquid.SquidOutput(tError);

        if (distanceToTarget > decelThreshold) {
            double translationMag = Math.hypot(strafePower, forwardPower);
            if (translationMag > 1e-4) {
                strafePower = (strafePower / translationMag) * powerScale;
                forwardPower = (forwardPower / translationMag) * powerScale;
            }
        }

        setWeightedMotorPowers(-forwardPower, -strafePower, turnPower);
    }

    public Pose2D getLocalizedPose() {
        return robotPose;
    }



    public static double wrapAngle360(double angle) {
        double wrapped = angle % 360.0;
        while (wrapped < 0) wrapped += 360.0;
        return wrapped;
    }

    public void update() {
        pinpoint.update();
        lastrobotPose = robotPose;
        robotPose = pinpoint.getPosition();
        x = robotPose.getX(DistanceUnit.INCH);
        y = robotPose.getY(DistanceUnit.INCH);
        head = wrapAngle360(robotPose.getHeading(AngleUnit.DEGREES));

        XVel = pinpoint.getVelX(DistanceUnit.INCH);
        YVel = pinpoint.getVelY(DistanceUnit.INCH);
        TVel = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

        switch (state) {
            case GO_TO_POINT:
                updateGoToPoint();
                break;
            case HOLD_POINT:
                break;

            case TELEOP:
                setWeightedMotorPowers(teleopStrafe, teleopForward, teleopTurn);
                break;

            case IDLE:
            default:
                setMotorPowers(0, 0, 0, 0);
                break;
        }
    }
}
