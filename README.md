# BLine-Lib

**BLine** is an open-source path generation and tracking suite designed for **holonomic drivetrains** (swerve, mecanum, etc.) made by students for students. It's built around simplicity and performance in time-constrained environments where quick iteration and rapid empirical testing prove advantageous.

📚 **[Documentation](https://bline-docs.pages.dev/)** — full guides, tutorials, and reference.

📖 **[Javadoc](https://edanliahovetsky.github.io/BLine-Lib/)** — full Java API documentation.

🎨 **[BLine-GUI](https://github.com/edanliahovetsky/BLine-GUI)** — visual path planning interface.

💬 **[Chief Delphi Thread](https://www.chiefdelphi.com/t/introducing-bline-a-new-rapid-polyline-autonomous-path-planning-suite/509778)** — discussion, feedback, and announcements.

![Robot Demo](docs/cone-demo.gif)

## Installation

### Using Vendor JSON (Recommended)

BLine-Lib's recommended WPILib vendor URL is the BLine Metrics Worker endpoint.
It serves the same vendor JSON as this repository while keeping aggregate fetch
counts for release health.

1. Open VS Code with your FRC project
2. Press `Ctrl+Shift+P` (or `Cmd+Shift+P` on Mac)
3. Type **"WPILib: Manage Vendor Libraries"**
4. Select **"Install new libraries (online)"**
5. Paste the recommended vendor URL:

```
https://bline-metrics.edan-liahovetsky.workers.dev/vendor/BLine-Lib.json
```

If the Worker URL is temporarily unavailable, use the direct GitHub vendor JSON
as the fallback:

```text
https://raw.githubusercontent.com/edanliahovetsky/BLine-Lib/main/BLine-Lib.json
```

### Using Gradle (Alternative)

Add JitPack repository to your `build.gradle`:

```gradle
repositories {
    maven { url 'https://jitpack.io' }
}
```

Add the dependency:

```gradle
dependencies {
    implementation 'com.github.edanliahovetsky:BLine-Lib:v0.9.1'
}
```

## Quick Start

For a complete getting started guide, see the **[Full Documentation](https://bline-docs.pages.dev/getting-started/)**.

### Basic Setup

```java
import frc.robot.lib.BLine.*;
import edu.wpi.first.math.controller.PIDController;

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
import edu.wpi.first.wpilibj2.command.Command;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

BLine has been validated through extensive testing with a WPILib physics simulation, utilizing Theta* for initial pathfinding and an Artificial Bee Colony (ABC) optimizer to benchmark the system against PathPlanner.

**Quantitative Results** from randomized Monte Carlo trials:

- **97% reduction** in path computation time
- **66% reduction** in cross-track error at waypoints
- Negligible **2.6% decrease** in total path tracking time compared to PathPlanner

**[Read the Full White Paper](https://docs.google.com/document/d/1Tc87YKWHtsEMEvmVDBD1Ww4e7vIUO2FyK3lwwuf-ZL4/edit?usp=sharing)**

## Building from Source

```bash
./gradlew build
```

Generate Javadoc locally:

```bash
./gradlew javadoc
# Open build/docs/javadoc/index.html
```

## Troubleshooting

If another robot repo consumes your local `BLine-Lib` checkout (for example via `includeBuild`) and you run `./gradlew clean` in this repo, you may need to rebuild the jar before the robot repo can package/sim successfully.

```bash
./gradlew jar
```

This regenerates `build/libs/BLine-Lib-<version>.jar` that some downstream fat-jar tasks expect.

## License

BSD 3-Clause License — See [LICENSE](LICENSE) file.
