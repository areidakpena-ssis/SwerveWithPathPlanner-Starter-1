# FRC Swerve Drivetrain Starter Code

Welcome to the starter repository for our team's **Swerve Drivetrain**. This codebase is designed as an educational starting point for high school robotics students learning WPILib Command-Based Java programming with CTRE Phoenix 6 hardware.

---

## Attribution & References

This project is a modified version of the **CTRE Phoenix 6 SwerveWithPathPlanner** starter project created by **Cross The Road Electronics (CTRE)**.

* **Original CTRE Source Repository:** [Phoenix6-Examples / SwerveWithPathPlanner](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithPathPlanner)
* **CTRE Phoenix 6 Documentation:** [Phoenix 6 Swerve Documentation](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html)
* **Swerve Module Hardware:** [SDS MK4i Swerve Modules](https://www.swervedrivespecialties.com/products/mk4i-swerve-module)

---

## Drivetrain Hardware Overview

Our swerve drive setup consists of the following components:
* **Swerve Modules:** 4× SDS MK4i Swerve Modules in the **L2** configuration with **Colson wheels**.
* **Drive Motors:** CTRE Talon FX (Integrated Brushless).
* **Steer (Azimuth) Motors:** CTRE Talon FX (Integrated Brushless).
* **Azimuth Absolute Encoders:** CTRE CANcoder.
* **Gyroscope / IMU:** CTRE Pigeon 2.0.
* **CAN Network:** Connected via the roboRIO CAN bus (`rio`).

---

## Repository & Code Structure (`src/`)

```text
src/main/java/frc/robot/
├── Main.java                         # Standard WPILib entry point
├── Robot.java                        # Robot lifecycle hooks (autonomousInit, teleopPeriodic, etc.)
├── RobotContainer.java               # Driver bindings, default commands, and subsystem setup
├── generated/
│   └── TunerConstants.java           # CAN IDs, gear ratios, PID gains, and physical dimensions
└── subsystems/
    └── CommandSwerveDrivetrain.java    # WPILib Subsystem wrapper for CTRE SwerveDrivetrain
```

---

## Key Files Deep-Dive

### 1. `TunerConstants.java`
**Purpose:** Generated initially via CTRE Phoenix Tuner X, this file centralizes all hardware identifiers, mechanical dimensions, sensor offsets, and motor controller PID/Feedforward gains.

#### CAN Bus Device Assignments (`rio` bus)
* **IMU / Gyroscope:** Pigeon 2.0 (ID: `20`)

| Module | Drive Motor ID | Steer Motor ID | Encoder (CANcoder) ID | Position (X, Y) |
| :--- | :---: | :---: | :---: | :---: |
| **Front Left** | `21` | `22` | `23` | (+10.75 in, +10.75 in) |
| **Front Right** | `24` | `25` | `26` | (+10.75 in, -10.75 in) |
| **Back Left** | `27` | `28` | `29` | (-10.75 in, +10.75 in) |
| **Back Right** | `30` | `31` | `32` | (-10.75 in, -10.75 in) |

#### Physical Robot Parameters
* **Drive Gear Ratio:** `6.746 : 1` (SDS MK4i L2 Gearing)
* **Steer Gear Ratio:** `150 / 7 : 1` (~`21.43 : 1`)
* **Wheel Radius:** `2.0 inches` (4-inch diameter Colson wheels)
* **Track Width & Wheel Base:** `21.5 inches` × `21.5 inches` (10.75-inch offset from robot center per axis)

---

### 2. `RobotContainer.java`
**Purpose:** Acts as the primary configuration hub for command-based programming. It instantiates the drivetrain, configures driver controllers, and maps joystick inputs/buttons to specific robot actions.

#### Core `SwerveRequest` Control Modes
CTRE Phoenix 6 uses `SwerveRequest` objects to define driving behaviors:
1. **`SwerveRequest.FieldCentric`**: Drives relative to the field (e.g., pushing forward on the stick moves the robot downfield regardless of robot heading).
2. **`SwerveRequest.RobotCentric`**: Drives relative to the robot's front bumper (standard arcade/tank steering behavior).
3. **`SwerveRequest.SwerveDriveBrake`**: Angles all four wheels inward into an "X" pattern to resist being pushed by opponent robots.
4. **`SwerveRequest.PointWheelsAt`**: Steers all four wheels to point in a specific direction without applying drive voltage.

#### Primary Controller & System Bindings
| Trigger / Input | Action / Command | Behavior & Swerve Request |
| :--- | :--- | :--- |
| **Left Stick (Y & X)** | Default Command: Field Translation | Controls forward/backward ($X$) and left/right ($Y$) movement using `SwerveRequest.FieldCentric`. |
| **Right Stick (X)** | Default Command: Field Rotation | Controls rotational rate ($\\theta$) counterclockwise/clockwise using `SwerveRequest.FieldCentric`. |
| **Left Bumper (`LB`)** | `seedFieldCentric()` | Resets the zero heading orientation for field-centric driving. |
| **Button `A`** | `SwerveDriveBrake` | Locks wheels in an "X" pattern while held to resist being pushed. |
| **Button `B`** | `PointWheelsAt` | Angles swerve modules toward the direction calculated by the left joystick vector without applying drive voltage. |
| **POV Up (D-Pad Up)** | Robot-Centric Forward | Drives straight forward at +0.5 m/s speed using `SwerveRequest.RobotCentric`. |
| **POV Down (D-Pad Down)** | Robot-Centric Reverse | Drives straight backward at -0.5 m/s speed using `SwerveRequest.RobotCentric`. |
| **Robot Disabled** | System Trigger: Neutral Mode | Applies `SwerveRequest.Idle` while disabled to maintain neutral motor mode configuration. |

#### SysId Characterization Routine Bindings
*(Hold trigger combination to execute routine; run each exactly once per log)*

| Trigger Combination | Routine Type | Direction | Command Executed |
| :--- | :--- | :--- | :--- |
| **Back + Y** | Dynamic | Forward | `drivetrain.sysIdDynamic(Direction.kForward)` |
| **Back + X** | Dynamic | Reverse | `drivetrain.sysIdDynamic(Direction.kReverse)` |
| **Start + Y** | Quasistatic | Forward | `drivetrain.sysIdQuasistatic(Direction.kForward)` |
| **Start + X** | Quasistatic | Reverse | `drivetrain.sysIdQuasistatic(Direction.kReverse)` |

---

### 3. `CommandSwerveDrivetrain.java`
**Purpose:** Wraps CTRE's low-level `SwerveDrivetrain` engine into a standard WPILib `Subsystem`, allowing easy integration with the WPILib Command framework.

#### Core Method: `applyRequest()`
```java
public Command applyRequest(Supplier<SwerveRequest> requestSupplier)
```
* **How it works:** `applyRequest()` takes a lambda function/supplier that delivers a `SwerveRequest` (such as driver joystick inputs updated continuously).
* **Why it matters:** It converts real-time control requests into a continuously executing WPILib `Command`. In `RobotContainer`, this is set as the drivetrain's **default command**, running every 20ms during teleoperated mode to smoothly update wheel speeds and angles.


---

## Student Verification Checklist (To-Dos)

### 1. Verify CAN Bus Assignments
- [ ] Open Phoenix Tuner X and connect to the robot via the roboRIO CAN bus (`rio`).
- [ ] Perform a device scan to confirm all 13 swerve devices (8 Talon FX motors, 4 CANcoders, 1 Pigeon 2.0) respond on the bus.
- [ ] Cross-reference physical device IDs with `TunerConstants.java`:
  - Pigeon 2.0: 20
  - Front Left: Drive 21, Steer 22, CANcoder 23
  - Front Right: Drive 24, Steer 25, CANcoder 26
  - Back Left: Drive 27, Steer 28, CANcoder 29
  - Back Right: Drive 30, Steer 31, CANcoder 32
- [ ] Perform a blink test in Phoenix Tuner X on each device to confirm physical module locations match code definitions.

### 2. Verify Physical Parameters of the Robot
- [ ] Measure physical wheel radius and update `kWheelRadius` in `TunerConstants.java` if tread wear deviates from the 2.0-inch nominal radius.
- [ ] Measure module offsets from frame center to ensure `kFrontLeftXPos`, `kFrontLeftYPos`, etc., accurately reflect physical dimensions (+/- 10.75 inches).
- [ ] Confirm drive motor gear ratio corresponds to SDS MK4i L2 gearing (6.746 : 1).
- [ ] Confirm steer motor gear ratio corresponds to 150 / 7 : 1 (~21.43 : 1).

### 3. Verify Steering Motor Gains (`steerGains`)
- [ ] Elevate the robot chassis securely on blocks so all wheels can rotate freely without ground contact.
- [ ] Review baseline closed-loop steer gains in `TunerConstants.java` (kP = 90, kI = 0, kD = 0.2, kS = 0.1, kV = 1.50).
- [ ] Enable the robot in Teleop mode and observe module responsiveness:
  - Check for high-frequency oscillation or jitter (indicates kP is too high or needs additional kD damping).
  - Check for sluggish response or static offset (indicates kP or kS feedforward is too low).
- [ ] Verify wheel zero alignment. Ensure all modules point straight forward at zero heading and update CANcoder offset constants in Phoenix Tuner X if necessary.