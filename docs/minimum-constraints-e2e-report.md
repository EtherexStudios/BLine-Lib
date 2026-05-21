# Minimum Constraints End-to-End Test Report

## Scope

This report covers the BLine minimum-constraint feature and its integration into a real FRC robot project through WPILib simulation. The feature adds minimum baseline constraints alongside the existing maximum caps, supports ranged minimum constraints by ordinal, parses those constraints from JSON, applies minimum velocity baselines in `FollowPath`, and falls back to global defaults with a warning when a minimum exceeds its paired maximum.

The tested BLine worktree was:

```text
/Users/edan/.codex/worktrees/e24d/BLine-Lib
```

The robot integration target was `FRC/2026-Robot-Code` from `origin/main` commit:

```text
18bfb7cb1248cbe882e8e854a7b9f369e5524163
```

The local `FRC/2026-Robot-Code` checkout had unrelated dirty files, so the e2e work was done in an isolated `wpilib-agent-tools` sandbox.

## Implementation Under Test

The BLine library changes under test were:

- `Path.WaypointConstraint`, `Path.TranslationTargetConstraint`, and `Path.RotationTargetConstraint` now carry minimum velocity fields in addition to maximum velocity and acceleration fields.
- `Path.PathConstraints` now supports global and ranged setters/getters for:
  - `min_velocity_meters_per_sec`
  - `min_velocity_deg_per_sec`
- Constraint resolution now pairs min/max values per ordinal. If `min > max`, the resolved value falls back to the default global maximum and disables the minimum baseline for that metric while logging a warning.
- `JsonUtils` now parses the new minimum constraint keys from path JSON.
- `FollowPath` applies minimum baseline enforcement for translation and rotation velocity outputs outside the corresponding end tolerance.
- `FollowPath` emits diagnostic telemetry for raw, max-clamped, and final controller outputs, plus resolved min/max values and whether the minimum baseline was applied.

## Local Library Verification

Commands run in the BLine worktree:

```bash
./gradlew compileJava
./gradlew test
```

Result:

```text
BUILD SUCCESSFUL
```

Coverage added at the library layer:

- `PathConstraintsTest.copyPreservesAllMinimumConstraintRanges`
- `PathConstraintsTest.rangedMinimumConstraintsResolveOnlyInsideTheirOrdinalRanges`
- `PathConstraintsTest.minimumGreaterThanMaximumWarnsAndFallsBackToGlobalDefault`
- `JsonUtilsTest` parsing coverage for both velocity `min_*` JSON keys
- `FollowPathTest.translationMinimumBaselineRaisesControllerOutputOutsideTolerance`
- `FollowPathTest.translationMinimumBaselineTurnsOffInsideTolerance`
- `FollowPathTest.rotationMinimumBaselineRaisesControllerOutputOutsideTolerance`
- `FollowPathTest.rotationMinimumBaselineTurnsOffInsideTolerance`

## WPILib Sandbox Setup

Sandbox created with:

```bash
/Users/edan/FRC/wpilib-agent-tools/scripts/run_cli.sh sandbox create \
  --name bline-min-e2e \
  --source branch:origin/main \
  --force \
  --json
```

Sandbox path:

```text
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e
```

Sandbox-only integration changes:

- `settings.gradle` substituted `com.github.edanliahovetsky:BLine-Lib` with the local BLine worktree through `includeBuild('/Users/edan/.codex/worktrees/e24d/BLine-Lib')`.
- `Constants.currentMode` was set to `Mode.SIM`.
- `Robot` enabled autonomous through `DriverStationSim` for deterministic simulation startup.
- `RobotContainer.getAutonomousCommand()` selected the e2e path from `BLINE_E2E_AUTO_PATH` and then called `Autos.followPath(pathName, true)`.
- Four synthetic path JSON files were added only inside the sandbox:
  - `bline_min_translation_range.json`
  - `bline_min_rotation_range.json`
  - `bline_min_conflict_fallback.json`
  - `bline_min_range_gap.json`

The robot project test suite was run in the sandbox:

```bash
/Users/edan/FRC/wpilib-agent-tools/scripts/run_cli.sh sandbox run \
  --name bline-min-e2e \
  -- ./gradlew test
```

Result:

```text
Completed sandbox run in 'bline-min-e2e' (exit_code=0)
BUILD SUCCESSFUL
```

Artifact:

```text
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/reports/sandbox-run-1779381693801.log
```

## End-to-End Simulation Matrix

Each simulation ran the robot code in WPILib sim, enabled autonomous mode, followed a BLine path through the robot's `Autos.followPath(...)` integration, recorded a WPILOG, and queried the resulting telemetry.

### 1. Translation Minimum Baseline In Range

Path:

```text
bline_min_translation_range.json
```

Intent:

- Translation ordinal 1 has `max_velocity_meters_per_sec = 1.0`.
- Translation ordinal 1 has `min_velocity_meters_per_sec = 0.8`.
- The target is close enough that the raw PID output is below the configured minimum while still outside the translation finish tolerance.

Log:

```text
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/logs/agent/logs/bline_min_translation_range.wpilog
```

Evidence:

- Driver Station state was autonomous and enabled.
- Swerve state transitioned `IDLE -> FOLLOW_PATH -> IDLE`.
- Resolved translation min/max telemetry:
  - `minTranslationVelocityMetersPerSec = 0.8`
  - `maxTranslationVelocityMetersPerSec = 1.0`
- Controller output telemetry:
  - First raw translation output: `0.3712976672012606`
  - First clamped translation output: `0.3712976672012606`
  - First final translation output: `0.8`
  - `translationMinimumApplied = true`
- Near the target, remaining distance dropped below the end tolerance:
  - Later remaining distance: `0.005889678966581879`
  - Later final output matched the clamped/raw value: `0.03121529852288396`
  - `translationMinimumApplied = false`

Conclusion:

The minimum translation baseline was applied only while outside tolerance, then stopped applying as the path reached its completion tolerance.

### 2. Rotation Minimum Baseline In Range

Path:

```text
bline_min_rotation_range.json
```

Intent:

- Rotation ordinal 1 has `max_velocity_deg_per_sec = 180`.
- Rotation ordinal 1 has `min_velocity_deg_per_sec = 90`.
- A small rotation target produces a raw rotational output below the minimum while outside rotation tolerance.

Log:

```text
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/logs/bline_min_rotation_range.wpilog
```

Evidence:

- Driver Station state was autonomous and enabled.
- Swerve state transitioned `IDLE -> FOLLOW_PATH -> IDLE`.
- Resolved rotation min/max telemetry:
  - `minRotationVelocityDegPerSec = 90.0`
  - `maxRotationVelocityDegPerSec = 180.0`
- Target rotation:
  - `targetRotationDeg = 5.729577951308233`
- Controller output telemetry:
  - First raw rotation output: `-0.01566414551985909` rad/s
  - First clamped rotation output: `-0.01566414551985909` rad/s
  - First final rotation output: `-1.5707963267948966` rad/s
  - `rotationMinimumApplied = true`
- After the constrained rotation segment, the resolved minimum returned to `0.0` and later samples used the raw/clamped rotational output.

Conclusion:

The minimum rotation baseline was applied with the correct sign and magnitude while outside tolerance, then released when the rotation target was no longer governed by that minimum range.

### 3. Conflicting Min/Max Fallback

Path:

```text
bline_min_conflict_fallback.json
```

Intent:

- Translation ordinal 1 has `max_velocity_meters_per_sec = 0.4`.
- Translation ordinal 1 has `min_velocity_meters_per_sec = 0.8`.
- Since the minimum exceeds the maximum, BLine should warn and resolve to the default global maximum with minimum disabled.

Log:

```text
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/logs/bline_min_conflict_fallback.wpilog
```

Evidence:

- Driver Station state was autonomous and enabled.
- Swerve state transitioned `IDLE -> FOLLOW_PATH -> IDLE`.
- The sim output included the expected warning:

```text
WARNING: Path constraint conflict for translation velocity at ordinal 1: minimum 0.8 exceeds maximum 0.4; using global default 4.5 and disabling the minimum baseline
```

- Resolved translation min/max telemetry:
  - `minTranslationVelocityMetersPerSec = 0.0`
  - `maxTranslationVelocityMetersPerSec = 4.5`
- Controller output telemetry showed no minimum enforcement:
  - Raw translation outputs included `0.4907`, `0.3075`, `0.1768`, `0.1020`, `0.0582`, and `0.0478`
  - Final translation outputs matched the raw/clamped values
  - `translationMinimumApplied = false`

Conclusion:

The conflict behavior matched the requested semantics: conflicting min/max constraints produced a warning, used the global default maximum, and disabled the minimum baseline.

### 4. Ranged Constraint Gap And Later Activation

Path:

```text
bline_min_range_gap.json
```

Intent:

- Translation ordinal 1 has no minimum baseline.
- Translation ordinal 2 has `max_velocity_meters_per_sec = 1.0`.
- Translation ordinal 2 has `min_velocity_meters_per_sec = 0.75`.
- This verifies that ranged minimum constraints do not leak into earlier ordinals and do activate later.

Log:

```text
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/logs/bline_min_range_gap.wpilog
```

Evidence:

- Driver Station state was autonomous and enabled.
- Swerve state transitioned `IDLE -> FOLLOW_PATH -> IDLE`.
- Resolved minimum telemetry moved from no minimum to the configured minimum:
  - First constrained sample: `minTranslationVelocityMetersPerSec = 0.0`
  - Later constrained sample: `minTranslationVelocityMetersPerSec = 0.75`
- Controller output telemetry:
  - Initial final output matched raw output with no minimum: `1.0327`
  - Later raw output was below the minimum: `0.3208`
  - Later final output was raised to `0.75`
  - `translationMinimumApplied` moved from `false` to `true`
  - At the end tolerance, `translationMinimumApplied` returned to `false`

Conclusion:

Ranged minimum constraints respected ordinal boundaries, activated only on the intended range, and still released inside the path completion tolerance.

## Query Artifacts

WPILOG key discovery and telemetry query artifacts:

```text
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/reports/sandbox-run-1779327007206.log
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/reports/sandbox-run-1779327036989.log
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/reports/sandbox-run-1779327067408.log
```

Simulation run artifacts:

```text
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/reports/sandbox-run-1779326914174.log
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/reports/sandbox-run-1779326935324.log
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/reports/sandbox-run-1779326953992.log
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/reports/sandbox-run-1779326971486.log
```

Sandbox patch artifact:

```text
/Users/edan/.wpilib-agent-tools/sandboxes/bline-min-e2e/agent/reports/bline-min-e2e-sandbox.patch
```

Note: the sandbox patch captures tracked robot-code edits. The synthetic path JSON files were sandbox-only untracked test inputs and are described in this report.

## Overall Result

All local library tests and all robot-project sandbox tests passed. The WPILib simulation evidence validates core behavior and edge cases:

- Minimum translation velocity baseline is applied outside tolerance.
- Minimum translation velocity baseline is released inside tolerance.
- Minimum rotation velocity baseline is applied outside tolerance with sign preserved.
- Minimum constraints are resolved by ordinal range.
- Ranged minimum constraints do not leak outside their configured ordinals.
- Conflicting `min > max` constraints warn, fall back to the global default maximum, and disable the minimum baseline.
- The feature integrates through the robot project's real autonomous path-following stack using `Autos.followPath(...)`.

No unexpected robot-code changes were made outside the isolated sandbox.
