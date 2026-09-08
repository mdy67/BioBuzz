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

    public final int currentPinpoint = 1;
    public State state = State.IDLE;
    public DcMotor leftFront, leftBack, rightBack, rightFront;
    public DcMotor[] driveMotors;
    GoBildaPinpointDriver pinpoint1, pinpoint2;
    public final List<DcMotor> driveMotorList = new ArrayList<>();

    private DcMotorEx initMotor(HardwareMap hw, String name, boolean reverse) {
        DcMotorEx motor = hw.get(DcMotorEx.class, name);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
    public double powerScale;

    public void updatePosData (int pinpoint) {
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

    public void goToPoint(double x, double y, double heading, double power, boolean predBrake, double xyThreshold, double hThreshold) {
        targetPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
        state = State.GO_TO_POINT;
        if (targetPose!= lastTarget) {
            atPoint = false;
            lastTarget = targetPose;
            targetX = x;
            targetY = y;
            targetT = heading;

            powerScale = power;
            this.xyThreshold = xyThreshold;
            this.hThreshold = hThreshold;
        }
    }

    public double xErr, yErr, tErr;
    private boolean atX, atY, atT;

    public void getErrors() {
        tErr = Utils.headingClip(targetT - head);
        double globxErr = (targetX - x);
        double globyErr = (targetY - y);

        xErr = globxErr * Math.cos(head) + globyErr * Math.sin(head);
        yErr = globyErr * Math.cos(head) + globxErr * Math.sin(head);

        atX = (Math.abs(xErr) <= xyThreshold / 2);
        atY = (Math.abs(yErr) <= xyThreshold / 2);
        atT = (Math.abs(tErr) <= hThreshold);

        atPoint = atX && atY && atT;

    }

    private double akP = 0.08;
    private double akD = 0.01;
    private double hkP = 0.14;
    private double hkD = 0.03;






}

