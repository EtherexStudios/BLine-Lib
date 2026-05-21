# BLine Rotation Override E2E Report

Generated: 2026-05-21

## Summary

This change adds a PathPlanner-style static rotation override to `FollowPath`.
The override supplier replaces BLine's normal rotation PID output and is called every
`execute()` loop while active. By default the supplied omega bypasses BLine's rotational
velocity and acceleration limiting so the caller owns the final path-follower omega
command. An explicit `RESPECT_CONSTRAINTS` mode keeps BLine's normal omega limits when
that is desired.

PathPlanner reference points:

- PathPlanner documents feedback overrides as static methods on `PPHolonomicDriveController`, with a supplier used for X, Y, or rotation during path following: https://pathplanner.dev/pplib-override-feedback.html
- The 2026 Java API exposes the analogous PathPlanner methods as
  `overrideRotationFeedback(DoubleSupplier)`, `clearRotationFeedbackOverride()`, and
  `clearFeedbackOverrides()`: https://pathplanner.dev/api/java/com/pathplanner/lib/controllers/PPHolonomicDriveController.html

## Implementation

Files changed in BLine:

- `src/main/java/frc/robot/lib/BLine/FollowPath.java`
- `src/test/java/frc/robot/lib/BLine/FollowPathTest.java`
- `README.md`

Added public API:

```java
FollowPath.overrideRotation(DoubleSupplier supplier);
FollowPath.overrideRotation(
    DoubleSupplier supplier,
    FollowPath.RotationOverrideBehavior behavior
);
FollowPath.clearRotationOverride();
```

Added behavior enum:

```java
FollowPath.RotationOverrideBehavior.RESPECT_CONSTRAINTS
FollowPath.RotationOverrideBehavior.BYPASS_CONSTRAINTS
```

Added telemetry:

- `FollowPath/rotationPidOutputRadPerSec`
- `FollowPath/rotationOverrideActive`
- `FollowPath/rotationOverrideBypassesConstraints`
- `FollowPath/rotationOverrideOmegaRadPerSec`
- `FollowPath/outputOmegaRadPerSec`

`FollowPath/rotationControllerOutput` remains the effective pre-limiter rotation,
which means it reports the override value when an override is active. The new
`rotationPidOutputRadPerSec` key preserves visibility into the original PID result.

## Unit Testing

Command:

```bash
./gradlew test
```

Result:

- Passed.
- Final run: `BUILD SUCCESSFUL in 1s`

Coverage added:

- Null supplier and null behavior are rejected.
- Normal PID rotation still runs when no override is active.
- Override supplier is sampled every `execute()` cycle.
- Default override behavior skips BLine's path-follower omega limiting.
- Explicit constrained mode respects path-follower omega limits.
- Clearing the override restores normal rotation for later commands.
- Degenerate zero-translation rotation path is used to isolate omega behavior.

Jar build:

```bash
./gradlew jar
```

Result:

- Passed.
- Final run: `BUILD SUCCESSFUL in 285ms`

## Robot E2E Setup

Robot repo requested by the test:

- Source repo: `/Users/edan/FRC/2026-Robot-Code`
- Main branch SHA used for sandbox: `18bfb7cb1248cbe882e8e854a7b9f369e5524163`
- Sandbox: `/Users/edan/.wpilib-agent-tools/sandboxes/bline-rotation-override-e2e`

The original robot repo was not edited. It already had local dirty path JSON files, so all
test changes were made in the wpilib-agent-tools sandbox.

Sandbox-only harness changes:

- `settings.gradle` uses `includeBuild('/Users/edan/.codex/worktrees/e130/BLine-Lib')`
  with dependency substitution for `com.github.edanliahovetsky:BLine-Lib`.
- `Constants.currentMode` is forced to `Mode.SIM`.
- `Robot.robotInit()` enables and sets autonomous mode through `DriverStationSim`.
- `RobotContainer.getAutonomousCommand()` checks `BLINE_E2E_SCENARIO`.
- A dedicated two-waypoint BLine path is created from `(3.0, 2.0, pi)` to
  `(6.0, 2.0, pi)`.
- The override supplier returns `20.0 rad/s` and logs `BLineE2E/suppliedOmegaRadPerSec`.
- `SwerveDrive` logs the omega received from BLine, the desired omega after subsystem
  override handling, and the obtainable omega after drivetrain limiting.

## E2E Commands

Sandbox creation:

```bash
/Users/edan/FRC/wpilib-agent-tools/scripts/run_cli.sh sandbox create \
  --name bline-rotation-override-e2e \
  --source branch:main \
  --force \
  --json
```

Sandbox compile:

```bash
/Users/edan/FRC/wpilib-agent-tools/scripts/run_cli.sh sandbox run \
  --name bline-rotation-override-e2e \
  -- ./gradlew compileJava
```

Scenario runs:

```bash
BLINE_E2E_SCENARIO=baseline_no_override \
/Users/edan/FRC/wpilib-agent-tools/scripts/run_cli.sh sandbox run \
  --name bline-rotation-override-e2e \
  -- sim --duration 25 --record-delay 0.5 \
  --record-output bline-rotation-baseline_no_override.wpilog --json

BLINE_E2E_SCENARIO=limited_override \
/Users/edan/FRC/wpilib-agent-tools/scripts/run_cli.sh sandbox run \
  --name bline-rotation-override-e2e \
  -- sim --duration 25 --record-delay 0.5 \
  --record-output bline-rotation-limited_override.wpilog --json

BLINE_E2E_SCENARIO=bypass_override \
/Users/edan/FRC/wpilib-agent-tools/scripts/run_cli.sh sandbox run \
  --name bline-rotation-override-e2e \
  -- sim --duration 25 --record-delay 0.5 \
  --record-output bline-rotation-bypass_override.wpilog --json

BLINE_E2E_SCENARIO=clear_override \
/Users/edan/FRC/wpilib-agent-tools/scripts/run_cli.sh sandbox run \
  --name bline-rotation-override-e2e \
  -- sim --duration 25 --record-delay 0.5 \
  --record-output bline-rotation-clear_override.wpilog --json
```

All four final simulation runs reached the requested 25-second duration. The raw simulator
exit code was `143`, normalized by wpilib-agent-tools to success because the bounded run
duration intentionally stops the simulator.

## E2E Artifacts

| Scenario | Wpilog | Run report |
| --- | --- | --- |
| `baseline_no_override` | `/Users/edan/.wpilib-agent-tools/sandboxes/bline-rotation-override-e2e/agent/logs/bline-rotation-baseline_no_override.wpilog` | `/Users/edan/.wpilib-agent-tools/sandboxes/bline-rotation-override-e2e/agent/reports/sandbox-run-1779371811044.log` |
| `limited_override` | `/Users/edan/.wpilib-agent-tools/sandboxes/bline-rotation-override-e2e/agent/logs/bline-rotation-limited_override.wpilog` | `/Users/edan/.wpilib-agent-tools/sandboxes/bline-rotation-override-e2e/agent/reports/sandbox-run-1779371843605.log` |
| `bypass_override` | `/Users/edan/.wpilib-agent-tools/sandboxes/bline-rotation-override-e2e/agent/logs/bline-rotation-bypass_override.wpilog` | `/Users/edan/.wpilib-agent-tools/sandboxes/bline-rotation-override-e2e/agent/reports/sandbox-run-1779371875567.log` |
| `clear_override` | `/Users/edan/.wpilib-agent-tools/sandboxes/bline-rotation-override-e2e/agent/logs/bline-rotation-clear_override.wpilog` | `/Users/edan/.wpilib-agent-tools/sandboxes/bline-rotation-override-e2e/agent/reports/sandbox-run-1779371905759.log` |

## DriverStation And Command Evidence

Every final log reported:

- `DriverStation/Enabled = true`
- `DriverStation/Autonomous = true`
- `DriverStation/Test = false`

Swerve system state evidence:

| Scenario | Swerve state sequence |
| --- | --- |
| `baseline_no_override` | `FOLLOW_PATH` at 0.194805s, `IDLE` at 0.704331s |
| `limited_override` | `FOLLOW_PATH` at 0.207120s, `IDLE` at 5.730286s |
| `bypass_override` | `FOLLOW_PATH` at 0.214491s, `IDLE` at 2.736350s |
| `clear_override` | `FOLLOW_PATH` at 0.208033s, `IDLE` at 2.516099s |

`FollowPath/finished` evidence:

| Scenario | Finish transition |
| --- | --- |
| `baseline_no_override` | false at 0.197872s, true at 0.704278s |
| `limited_override` | false at 0.210400s, true at 5.730293s |
| `bypass_override` | false at 0.217522s, true at 2.736358s |
| `clear_override` | false at 0.210887s, true at 2.516107s |

## Omega Evidence

The key distinction is between BLine's final path-follower omega and the downstream
swerve subsystem's obtainable omega. Bypass mode bypasses the BLine path-follower limiter,
but the drivetrain can still saturate the physical module setpoints later.

| Scenario | Override active | Bypass active | Supplier max rad/s | BLine output omega max rad/s | Swerve received omega max rad/s | Swerve obtainable omega max rad/s |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `baseline_no_override` | false | false | 0.000000 | 0.000000 | 0.000000 | 0.000000 |
| `limited_override` | true | false | 20.000000 | 10.471976 | 10.471976 | 10.471976 |
| `bypass_override` | true | true | 20.000000 | 20.000000 | 20.000000 | 12.240000 |
| `clear_override` | true, then false | true, then false | 20.000000 | 20.000000 | 20.000000 | 12.240000 |

Additional observations:

- `baseline_no_override` proves normal path following does not activate or leak an override.
- `limited_override` proves the static supplier is integrated and sampled, while explicit
  `RESPECT_CONSTRAINTS` still applies BLine path constraints.
- `bypass_override` proves the same supplier reaches the swerve subsystem as `20.0 rad/s`
  when the default one-argument override API is used.
- `clear_override` proves an active override can be cleared mid-command. The active flag
  changed from true at 0.210874s to false at 1.221339s, and the bypass flag changed from
  true at 0.210878s to false at 1.221350s. After that point, BLine output follows normal
  rotation PID correction values instead of the fixed supplier.

Path progress evidence from `FollowPath/remainingPathDistanceMeters`:

| Scenario | Sample count | Min meters | Max meters |
| --- | ---: | ---: | ---: |
| `baseline_no_override` | 6 | 0.014115 | 1.642084 |
| `limited_override` | 56 | 0.000000 | 2.993264 |
| `bypass_override` | 26 | 0.003141 | 2.995218 |
| `clear_override` | 24 | 0.002343 | 2.992876 |

## Edge Cases Covered

- No override registered.
- Override registered with default bypass behavior.
- Override registered with explicit constrained behavior.
- Override cleared during a running path.
- Supplier output much larger than the normal omega limit (`20.0 rad/s`).
- Zero-translation unit-test path isolates rotation behavior.
- Robot integration confirms the final BLine omega is the value received by the swerve
  subsystem before downstream drivetrain limiting.

## Run Notes

- The first e2e attempt failed because the robot repo's jar task expected
  `/Users/edan/.codex/worktrees/e130/BLine-Lib/build/libs/BLine-Lib-0.8.4.jar`.
  Running `./gradlew jar` in BLine produced the jar and subsequent sandbox runs passed.
- An initial test using the robot repo's `straight` path ended almost immediately because
  that path only had one waypoint. The final evidence uses the dedicated two-waypoint
  sandbox path described above.
- A temporary NT4 port conflict appeared while another sandbox was active. I waited for
  port `5810` to clear rather than terminating unrelated work.
- The simulator reported joystick-unplugged warnings and first-time AdvantageKit struct
  logging warnings. These did not block autonomous enablement, command execution, or log
  generation.
- After the final e2e runs, `lsof -nP -iTCP:5810 -sTCP:LISTEN` returned no listener.

## Result

The new BLine rotation override passes unit testing and robot-repo e2e simulation
against `FRC/2026-Robot-Code` main. The evidence confirms the core behavior, the default
bypass behavior, the opt-in constrained behavior, clearing behavior, and integration into
the swerve subsystem command path.

Residual risk:

- This was simulation-only, not real hardware.
- The override is static and applies to all active `FollowPath` commands, matching the
  PathPlanner model. Callers must clear it when they no longer want the override.
- The one-argument override API bypasses BLine's path-follower omega limiting, but downstream
  drivetrain code can still limit or saturate the physically obtainable omega.
