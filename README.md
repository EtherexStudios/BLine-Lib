<h1 align="center">BLine-Lib</h1>

<p align="center">
  <a href="BLine-Lib-2027.json"><img src="https://img.shields.io/badge/version-0.9.1--wpilib2027.alpha06.01-2563eb" alt="Compatibility version 0.9.1-wpilib2027.alpha06.01"></a>
  <a href="BLine-Lib-2027.json"><img src="https://img.shields.io/badge/WPILib-2027.0.0--alpha--6-c1121f" alt="WPILib 2027.0.0 Alpha 6"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-BSD--3--Clause-0f766e" alt="BSD 3-Clause License"></a>
</p>

**BLine** is a rapid point-to-point autonomous path planning and tracking
library for FIRST Robotics Competition. It is made by students for students and
built around practical tuning, quick iteration, and rapid empirical testing in
time-constrained build-season environments.

> [!IMPORTANT]
> This is the `wpilib-2027` compatibility line for WPILib 2027.0.0-alpha-6 and
> Commands v2. Read the [WPILib 2027 installation, migration, and validation
> guide](WPILIB_2027.md) before using it. The `main` branch remains the stable
> WPILib 2026 release line.

**Quick links**

🚀 **[Open the hosted editor](https://bline-web.pages.dev/)** — create, tune, preview, and export BLine paths in the browser.

🖥️ **[BLine Web](https://github.com/edanliahovetsky/BLine-Web)** — current web and desktop editor.

💬 **[Chief Delphi Thread](https://www.chiefdelphi.com/t/introducing-bline-a-new-rapid-polyline-autonomous-path-planning-suite/509778)** — discussion, feedback, and announcements.

📚 **[Documentation](https://bline-docs.pages.dev/)** — stable WPILib 2026 guides, tutorials, and reference.

<p align="center">
  <img src="docs/readme/bline-web-demo.gif" alt="BLine Web editor GUI demo" width="900">
  <br><br>
  <img src="docs/cone-demo.gif" alt="BLine robot cone demo" width="900">
</p>

## Installation

Use the year-specific `BLine-Lib-2027.json`, not the stable `BLine-Lib.json`.
The [WPILib 2027 guide](WPILIB_2027.md) gives the current pre-publication URL,
dependency provenance, Commands v2 requirement, and migration steps. No
immutable compatibility tag or Formal Release exists yet.

## Quick Start

For BLine API setup after the compatibility vendordep is installed, see the
**[getting started guide](https://bline-docs.pages.dev/getting-started/)**. Its
installation steps and WPILib imports target stable WPILib 2026; use the
[WPILib 2027 guide](WPILIB_2027.md) for those year-specific steps.

### Basic Setup

```java
import frc.robot.lib.BLine.*;
import org.wpilib.math.controller.PIDController;

// 1. Set global constraints
Path.setDefaultGlobalConstraints(new Path.DefaultGlobalConstraints(
    4.0,    // maxVelocityMetersPerSec
    3.0,    // maxAccelerationMetersPerSec2
    360.0,  // maxVelocityDegPerSec
    720.0,  // maxAccelerationDegPerSec2
    0.05,   // endTranslationToleranceMeters
    2.0,    // endRotationToleranceDeg
    0.3     // intermediateHandoffRadiusMeters
));

// 2. Create a reusable path builder
FollowPath.Builder pathBuilder = new FollowPath.Builder(
    driveSubsystem,
    driveSubsystem::getPose,
    driveSubsystem::getChassisSpeeds,
    driveSubsystem::drive,
    new PIDController(5.0, 0.0, 0.0),  // translation
    new PIDController(3.0, 0.0, 0.0),  // rotation
    new PIDController(2.0, 0.0, 0.0)   // cross-track
).withDefaultShouldFlip()
 .withPoseReset(driveSubsystem::resetPose);

// 3. Load and follow a path
Path myPath = new Path("myPathFile");  // loads deploy/autos/paths/myPathFile.json
Command followCommand = pathBuilder.build(myPath);
```

### Rotation Override

For paths that need another system to own robot heading temporarily, such as
vision aiming while translating along a path, override the path follower's
rotational output:

```java
FollowPath.overrideRotation(
    () -> shooterAimController.getOmegaRadiansPerSecond()
);

// Later, when normal path rotation should resume:
FollowPath.clearRotationOverride();
```

The default override behavior bypasses BLine's rotational velocity and
acceleration constraints so the caller owns the final path-follower omega
command. If the supplied omega should still respect BLine's rotation limits,
use the explicit constrained mode:

```java
FollowPath.overrideRotation(
    () -> shooterAimController.getOmegaRadiansPerSecond(),
    FollowPath.RotationOverrideBehavior.RESPECT_CONSTRAINTS
);
```

### Command-Based Autos With Event Triggers

When a BLine path contains event triggers that schedule WPILib commands, prefer
`BLineCommands` for the surrounding command composition:

```java
import static frc.robot.lib.BLine.BLineCommands.sequence;
import org.wpilib.command2.Command;

Command auto = sequence(
    shooter.shoot().withTimeout(2.0),
    pathing.followPath("intakethroughdepot"),
    shooter.shoot()
);
```

`BLineCommands` mirrors the WPILib `Commands` composition methods that accept
child commands, but proxies those children before building the group. This keeps
the outer auto from owning every child requirement for its whole lifetime, which
lets BLine event-trigger commands use normal WPILib scheduling. See the
`BLineCommands` Javadocs for method-by-method behavior and limitations. The API
intentionally contains only WPILib `Commands` counterparts: `either`, `select`,
`defer`, `deferredProxy`, `sequence`, `repeatingSequence`, `parallel`, `race`,
and `deadline`.

### Field2d Visualization

BLine paths are polylines, so visualizing them on a WPILib `Field2d` widget (in
Elastic or Glass) needs no simulation. `BLineField` provides small helpers for
drawing a BLine path directly as a connected field object:

```java
import frc.robot.lib.BLine.BLineField;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;

Field2d field = new Field2d();
SmartDashboard.putData("Field", field);

// Draw a planned path once. BLine assigns a stable unique field object name for
// this path instance and returns it if you want to inspect it.
String objectName = BLineField.drawPath(field, myPath);

// Or choose the display slot yourself. BLine appends "Trajectory" if needed.
BLineField.drawPath(field, "ScoreTwo", myPath);
```

The no-name overload generates names like `"BLinePath0Trajectory"` per
`Field2d`, reusing the same name when called again with the same `Path` instance.
Explicit names are treated as user-owned display slots, so reusing
`"ScoreTwo"` updates `"ScoreTwoTrajectory"`. If you only need the raw polyline
points, use `myPath.getTranslations()`.

## Performance

BLine has been validated with randomized Monte Carlo trials in a WPILib physics
simulation, using Theta* for initial pathfinding and an Artificial Bee Colony
(ABC) optimizer to benchmark the system against PathPlanner.

| Measurement | Result |
| --- | --- |
| Path computation time | **97% reduction** |
| Cross-track error at waypoints | **66% reduction** |
| Total path tracking time | **2.6% decrease** compared to PathPlanner |

Read the **[full white paper](https://docs.google.com/document/d/1Tc87YKWHtsEMEvmVDBD1Ww4e7vIUO2FyK3lwwuf-ZL4/edit?usp=sharing)**.

## Build From Source

```bash
./gradlew build
```

### API Reference

Generate Javadoc locally:

```bash
./gradlew javadoc
# Open build/docs/javadoc/index.html
```

Published API reference: **[Javadoc](https://edanliahovetsky.github.io/BLine-Lib/)**
(stable WPILib 2026 only). Compatibility artifacts retain their own Java 25
Javadoc and sources jars; see the [WPILib 2027 guide](WPILIB_2027.md).

## Troubleshooting

If another robot repo consumes your local `BLine-Lib` checkout (for example via
`includeBuild`) and you run `./gradlew clean` in this repo, rebuild the jar
before the robot repo packages or runs simulation.

```bash
./gradlew jar
```

This regenerates `build/libs/BLine-Lib-<version>.jar` for downstream fat-jar
tasks.

## License

BSD 3-Clause License. See [LICENSE](LICENSE).
