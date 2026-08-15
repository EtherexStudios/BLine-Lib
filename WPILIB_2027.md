# BLine-Lib for WPILib 2027 Alpha 6

The `wpilib-2027` branch adapts BLine-Lib 0.9.1 to
WPILib `2027.0.0-alpha-6` and Java 25. It is a compatibility line for the same
BLine path behavior, not an independent feature fork. The stable, default
release line remains [`main`](https://github.com/edanliahovetsky/BLine-Lib)
for WPILib 2026.

This line uses **Commands v2 only**. Commands v3 is not supported.

## Install the current pre-publication build

There is not yet an immutable compatibility tag or GitHub Formal Release. The
current vendordep is a commit-addressed validation build so teams can inspect
and exercise the exact source that passed the pre-publication checks.

1. Remove the stable BLine vendordep and any Commands v3 vendordep from the
   robot project.
2. In WPILib VS Code, run **WPILib: Manage Vendor Libraries** and choose
   **Install new libraries (online)**.
3. Install this compatibility manifest:

   ```text
   https://raw.githubusercontent.com/edanliahovetsky/BLine-Lib/wpilib-2027/BLine-Lib-2027.json
   ```

4. Confirm the project resolves BLine-Lib and
   `org.wpilib.commandsv2:commandsv2-java:2027.0.0-alpha-6`, then compile the
   robot project.

The manifest declares `wpilibYear` as `2027_alpha5`. That is the WPILib tooling
identifier and local installation directory used by the Alpha 6 release; it
does not mean BLine targets WPILib Alpha 5. The 2026 and 2027 manifests retain
the same BLine UUID, so WPILib treats them as the same library and they cannot
coexist in one robot project.

The current manifest resolves BLine-Lib from exact source commit
`4dd378b77a0ec73d4c89efb752756728d138801a`. This is intentionally different
from the later release installation flow: after the Publication gate, the
vendordep will point to the exact immutable compatibility tag, and a separate
Human acceptance pass will install that tagged vendordep in a pristine project.

## Migrate from the stable line

Keep the same path JSON, BLine constraints, field-flipping choices, and
Commands v2 scheduling model. The compatibility work preserves BLine-owned
behavior and defaults; it does not add compatibility features or deliberate
behavior changes.

WPILib 2027 moves Java packages from `edu.wpi.first` to `org.wpilib`. Update
your robot-project WPILib imports accordingly. For example:

```java
import org.wpilib.math.controller.PIDController;
import org.wpilib.command2.Command;
```

BLine's public package remains `frc.robot.lib.BLine`. Review compiler errors
for other upstream WPILib type or package changes in your robot code, and keep
the robot project on Java 25.

## Compatibility versioning

Compatibility releases retain the shared BLine base version and add a
zero-padded WPILib alpha and compatibility revision:

```text
v0.9.1-wpilib2027.alpha06.01       immutable Git tag
0.9.1-wpilib2027.alpha06.01        vendordep version
```

The leading `v` is omitted from the vendordep version because WPILib compares
that field as a version. A compatibility-only rebuild increments the final
two-digit revision. A later WPILib alpha increments the two-digit alpha field.
Failed candidate tags remain immutable, so a retry receives the next revision.

Alpha and beta GitHub Releases are prereleases. A tag, JitPack build, or
development-branch push alone is not a Formal Release.

## Validation evidence

Both maintained lines run parity tests through BLine's public API using real
path JSON and the real Commands v2 scheduler. The coverage includes field
geometry and flipping, constraints, and defaults. GitHub Actions also runs
`build publishToMavenLocal` on pushes and pull requests: Java 17 with WPILib
2026 on `main`, and Java 25 with WPILib Alpha 6 on `wpilib-2027`. These
informational workflows are not required protection checks.

The current compatibility dependency was also compiled in a clean consumer
with no Maven Local, composite build, or project dependency. A local-only real
WPILib 2027 RobotCode project then loaded a deployed BLine path and scheduled
`FollowPath` with Commands v2. Its final deterministic run passed all five
numeric assertions: DriverStation autonomous-enabled, finite output, overall
verdict, endpoint error `0.038710 m` within the `0.08 m` limit, and stopped
output exactly `0`. It recorded 82 samples over 1.592 seconds.

## Validation limits

The RobotCode harness integrates commanded chassis velocity into pose feedback
at a fixed 20 ms step. It proves that the real dependency installs, a real path
loads, Commands v2 schedules BLine, finite output reaches the endpoint within
the timeout, and output stops afterward.

It does **not** model drivetrain or module dynamics, traction, latency, sensor
noise, closed-loop drivetrain controllers, Systemcore hardware, or a physical
robot. It is not drivetrain-performance or real-robot validation.

## Documentation and artifacts

The generated BLine documentation site and published Pages Javadocs remain the
stable WPILib 2026 documentation surface. Compatibility artifacts still include
their Java 25 sources and Javadoc jars so the exact compatibility API can be
inspected without replacing the stable site.
