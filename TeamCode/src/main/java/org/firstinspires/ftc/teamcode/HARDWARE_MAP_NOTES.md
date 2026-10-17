# Robot Hardware Configuration & Mapping Notes

This document provides a reference for all motor ports, servo ports, encoder slots, and hardware names configured in the robot code.

## 1. Drivetrain
- **Left Front Motor**: `DcMotorEx` | Name: `"leftFront"` | Reversed: `false`
- **Left Back Motor**: `DcMotorEx` | Name: `"leftBack"` | Reversed: `false`
- **Right Back Motor**: `DcMotorEx` | Name: `"rightBack"` | Reversed: `true`
- **Right Front Motor**: `DcMotorEx` | Name: `"rightFront"` | Reversed: `true`
- **Pinpoint Odometry Driver 1**: `GoBildaPinpointDriver` | Name: `"pinpoint1"`
- **Pinpoint Odometry Driver 2**: `GoBildaPinpointDriver` | Name: `"pinpoint2"`

## 2. Shooter
- **Left Flywheel Motor**: `DcMotorEx` | Name: `"flywheelL"`
- **Right Flywheel Motor**: `DcMotorEx` | Name: `"flywheelR"` (Reversed)
- **Flywheel Encoder**: REV Through Bore / Motor Encoder connected to `flywheelR` motor encoder port (used for RPM velocity PID).

## 3. Intake
- **Intake Motor**: `DcMotor` | Name: `"intake"`
  - **Encoder Slot**: The encoder port on `"intake"` is used by the **Turret REV Through Bore Encoder** (8192 counts/rev).
- **Transfer Motor**: `DcMotor` | Name: `"transfer"`

## 4. Turret
- **Turret Rotation Servo**: `CRServo` | Name: `"turretServo"`
- **Turret Encoder**: `DcMotorEx` | Plugged into motor port `"intake"` (8192 counts/rev)
- **Hood Servo**: `Servo` | Name: `"hoodServo"` (Range constrained: `0.2` to `0.5`)

## 5. Summary Table

| Subsystem | Component | Device Type | Hardware Name | Encoder Slot / Notes |
| :--- | :--- | :--- | :--- | :--- |
| Drivetrain | Left Front | `DcMotorEx` | `"leftFront"` | Wheel Encoder |
| Drivetrain | Left Back | `DcMotorEx` | `"leftBack"` | Wheel Encoder |
| Drivetrain | Right Back | `DcMotorEx` | `"rightBack"` | Wheel Encoder |
| Drivetrain | Right Front | `DcMotorEx` | `"rightFront"` | Wheel Encoder |
| Drivetrain | Pinpoint 1 | `GoBildaPinpointDriver` | `"pinpoint1"` | Odometry Pod |
| Drivetrain | Pinpoint 2 | `GoBildaPinpointDriver` | `"pinpoint2"` | Odometry Pod |
| Shooter | Left Flywheel | `DcMotorEx` | `"flywheelL"` | Slave / Coupled |
| Shooter | Right Flywheel | `DcMotorEx` | `"flywheelR"` | **Flywheel RPM Encoder** |
| Intake | Intake Motor | `DcMotor` | `"intake"` | **Turret Encoder** plugged into this port |
| Intake | Transfer Motor | `DcMotor` | `"transfer"` | - |
| Turret | Turret Servo | `CRServo` | `"turretServo"` | Positioned by PID |
| Turret | Turret Encoder | `DcMotorEx` | `"intake"` | REV Through Bore (8192 CPR) |
| Turret | Hood Servo | `Servo` | `"hoodServo"` | Angle Range: `0.2` to `0.5` |
