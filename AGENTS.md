# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project Overview

FRC 2026 robot code for Team 8810. Java-based, built on WPILib Command-Based Framework with AdvantageKit logging system. The robot features a swerve drive, dual-flywheel shooter with adjustable hood, feeder/indexer ball path, deployable intake, hopper sensors, and LED indicators.

## Build & Development Commands

```bash
# Build the project
./gradlew build

# Deploy to RoboRIO
./gradlew deploy

# Run tests (JUnit 5)
./gradlew test

# Run a single test class
./gradlew test --tests "frc.robot.SomeTest"

# Format code (auto-runs on build, but can be run manually)
./gradlew spotlessApply

# Check formatting without applying
./gradlew spotlessCheck

# Simulate (AdvantageKit sim, GUI disabled by default)
./gradlew simulateJava

# Log replay
./gradlew replayWatch
```

Build toolchain: GradleRIO 2026.2.1, Java 17, Spotless (Google Java Format).
Spotless runs automatically before compile (`compileJava.dependsOn(spotlessApply)`), so code is always formatted on build.

## Architecture

### Three-Mode IO Abstraction Pattern

Every subsystem uses an IO interface layer enabling three runtime modes (`Constants.Mode`):

- **REAL** — Hardware IO implementations (e.g., `ShooterIOTalonFX`, `GyroIOPigeon2`)
- **SIM** — Physics simulation implementations (e.g., `ModuleIOSim`)
- **REPLAY** — Empty anonymous implementations (`new ShooterIO() {}`) for log replay

Each subsystem follows this structure:
```
subsystems/{name}/
├── {Name}.java          # Subsystem class (extends SubsystemBase)
├── {Name}IO.java        # IO interface with @AutoLog Inputs inner class
└── {Name}IOTalonFX.java # Real hardware implementation (Phoenix 6)
```

The `@AutoLog` annotation on the `Inputs` inner class auto-generates logging wrappers via AdvantageKit's annotation processor. Subsystem constructors accept the IO interface, and `RobotContainer` selects the implementation based on `Constants.currentMode`.

### Subsystems

| Subsystem | Description | Hardware |
|-----------|-------------|----------|
| Drive | Swerve drivetrain (4 modules) | TalonFX + Pigeon2 |
| Shooter | Dual flywheel pairs (2 leaders + 2 followers) | TalonFX (IDs 13-16) |
| Hood | Shooter angle adjustment | TalonFX (ID 21) |
| Feeder | Ball feeding to shooter | TalonFX (ID 17 + follower 18) |
| Indexer | Ball sorting/staging | TalonFX (IDs 19-20) |
| Intake | Deployable ball intake with rollers | TalonFX (IDs 22-23) |
| Hopper | Ball presence detection | CANrange sensors |
| LED | Status indicators | CANdle |

### Key Command Patterns

- **MegaTrackIterativeCommand** — Moving-shot with iterative flight-time compensation. Uses fixed-point iteration to predict ball landing position while robot moves.
- **SmashBumpCommand / SmashTrenchCommand** — Obstacle traversal with alignment phases (ALIGN → RUN/SPRINT), using Autopilot for path planning and heading hold PID.
- **DriveCommands** — Static factory methods for drive behaviors (joystick drive, feedforward characterization, etc.)
- **Auto commands** (Down, DownMagic, UpOut) — Autonomous routines in `commands/Auto/`

### Constants Organization

`Constants.java` uses nested static inner classes to group related constants:
- `FieldConstants` — Field geometry, hub locations, trench/bump approach lines with mirror symmetry helpers
- `AutoShootConstants` — Hood angle and shooter speed interpolation maps (distance → setting)
- `VisionConstants` — Limelight MegaTag2 std dev tuning
- `{Subsystem}Constants` — CAN IDs, PID gains, current limits per subsystem
- Command-specific constants classes (e.g., `MegaTrackCommandConstants`, `BumpCommandConstants`)

### Vendor Dependencies

- **AdvantageKit 26.0.0** — Logging, replay, and `@AutoLog`
- **Phoenix 6 (26.1.0)** — CTRE motor controllers (TalonFX), Pigeon2 gyro, CANdle, CANrange
- **PathPlanner** — Autonomous path planning and trajectory following
- **Autopilot** — Point-to-point navigation with motion profiling
- **Studica** — Additional hardware support

### Configuration

- `TunerConstants` (in `generated/`) — Phoenix 6 Tuner-generated swerve module constants. Do not hand-edit.
- `.wpilib/wpilib_preferences.json` — Team number (8810)
- `src/main/deploy/` — Path planning data files (Choreo, PathPlanner)

### Runtime Tuning

`LoggedTunableNumber` provides NetworkTables-based real-time parameter tuning. Values are logged through AdvantageKit and editable via dashboard during development.

### Event Deploy

On branches prefixed with `event`, the build system auto-commits all changes before deploy (`eventDeploy` task). This captures the exact deployed code state during competitions.

## Code Conventions

- License: GPL-3.0 header on all source files (FRC 6328 template)
- Formatting: Google Java Format (enforced by Spotless)
- Naming: Standard Java camelCase; static constants UPPER_SNAKE_CASE
- Subsystem state machines use `WantedState`/`SystemState` enums (see Intake)
- InterpolatingDoubleTreeMap for distance-based lookup tables (hood angle, shooter speed, flight time)
- Alliance-aware field geometry: blue-side coordinates as source of truth, mirrored for red via helper methods
