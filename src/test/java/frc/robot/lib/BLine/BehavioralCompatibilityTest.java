package frc.robot.lib.BLine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.PathElementConstraint;
import frc.robot.lib.BLine.Path.TranslationTargetConstraint;
import java.io.File;
import java.net.URISyntaxException;
import java.net.URL;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class BehavioralCompatibilityTest {
    private static final double EPSILON = 1e-9;
    private static final List<Translation2d> EXPECTED_TRANSLATIONS = List.of(
        new Translation2d(1.25, 2.50),
        new Translation2d(4.75, 3.25),
        new Translation2d(7.00, 1.50)
    );

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    @AfterEach
    void tearDown() {
        scheduler.cancelAll();
        scheduler.clearComposedCommands();
        scheduler.enable();
        FlippingUtil.fieldSizeX = 16.54;
        FlippingUtil.fieldSizeY = 8.07;
        FlippingUtil.symmetryType = FlippingUtil.FieldSymmetry.kRotational;
    }

    @Test
    void representativePathLoadsGeometryConstraintsAndDefaultsFromRealFiles() throws URISyntaxException {
        Path path = loadRepresentativePath();

        assertTrue(path.isValid());
        assertEquals(EXPECTED_TRANSLATIONS, path.getTranslations());
        assertEquals(0.04, path.getEndTranslationToleranceMeters(), EPSILON);
        assertEquals(1.75, path.getEndRotationToleranceDeg(), EPSILON);

        Path.DefaultGlobalConstraints defaults = path.getDefaultGlobalConstraints();
        assertEquals(4.80, defaults.getMaxVelocityMetersPerSec(), EPSILON);
        assertEquals(6.40, defaults.getMaxAccelerationMetersPerSec2(), EPSILON);
        assertEquals(690.0, defaults.getMaxVelocityDegPerSec(), EPSILON);
        assertEquals(1320.0, defaults.getMaxAccelerationDegPerSec2(), EPSILON);
        assertEquals(0.06, defaults.getEndTranslationToleranceMeters(), EPSILON);
        assertEquals(2.25, defaults.getEndRotationToleranceDeg(), EPSILON);
        assertEquals(0.28, defaults.getIntermediateHandoffRadiusMeters(), EPSILON);

        List<Pair<PathElement, PathElementConstraint>> resolved = path.getPathElementsWithConstraintsNoWaypoints();
        List<TranslationTargetConstraint> translationConstraints = resolved.stream()
            .filter(pair -> pair.getFirst() instanceof Path.TranslationTarget)
            .map(pair -> (TranslationTargetConstraint) pair.getSecond())
            .toList();
        TranslationTargetConstraint firstTranslation = translationConstraints.get(0);
        TranslationTargetConstraint secondTranslation = translationConstraints.get(1);
        assertEquals(2.40, firstTranslation.maxVelocityMetersPerSec(), EPSILON);
        assertEquals(3.60, firstTranslation.maxAccelerationMetersPerSec2(), EPSILON);
        assertEquals(0.55, firstTranslation.minVelocityMetersPerSec(), EPSILON);
        assertEquals(3.10, secondTranslation.maxVelocityMetersPerSec(), EPSILON);
    }

    @Test
    void publicBuilderUsesConfiguredFieldDimensionsForFlipAndMirror() throws URISyntaxException {
        FlippingUtil.fieldSizeX = 18.0;
        FlippingUtil.fieldSizeY = 9.0;

        MutableRobot flippedRobot = new MutableRobot();
        Command flippedCommand = builder(flippedRobot)
            .withShouldFlip(() -> true)
            .withPoseReset(flippedRobot::setPose)
            .build(loadRepresentativePath())
            .ignoringDisable(true);

        scheduleAndRunOnce(flippedCommand);
        assertPose(flippedRobot.pose, 16.75, 6.50, -150.0);

        scheduler.cancelAll();
        scheduler.clearComposedCommands();

        MutableRobot mirroredRobot = new MutableRobot();
        Command mirroredCommand = builder(mirroredRobot)
            .withShouldMirror(() -> true)
            .withPoseReset(mirroredRobot::setPose)
            .build(loadRepresentativePath())
            .ignoringDisable(true);

        scheduleAndRunOnce(mirroredCommand);
        assertPose(mirroredRobot.pose, 1.25, 6.50, -30.0);
    }

    @Test
    void followerObservesMutableRobotPoseThroughRealCommandScheduler() throws URISyntaxException {
        MutableRobot robot = new MutableRobot();
        FollowPath follower = builder(robot)
            .withPoseReset(robot::setPose)
            .build(loadRepresentativePath());
        Command scheduledFollower = follower.ignoringDisable(true);
        AtomicBoolean markerRan = new AtomicBoolean(false);
        FollowPath.registerEventTrigger(
            "behavior-parity-marker",
            Commands.runOnce(() -> markerRan.set(true)).ignoringDisable(true)
        );

        scheduler.schedule(scheduledFollower);
        scheduler.run();

        assertTrue(scheduler.isScheduled(scheduledFollower));
        assertPose(robot.pose, 1.25, 2.50, 30.0);
        assertTrue(
            Math.hypot(robot.commandedSpeeds.vxMetersPerSecond, robot.commandedSpeeds.vyMetersPerSecond) > 0.0,
            "Follower should command translation while the mutable pose is at the path start"
        );

        robot.setPose(new Pose2d(4.75, 3.25, Rotation2d.fromDegrees(90.0)));
        for (int cycle = 0; cycle < 3; cycle++) {
            scheduler.run();
        }
        assertTrue(markerRan.get(), "Loaded event marker should schedule a real Commands v2 command");

        robot.setPose(new Pose2d(7.00, 1.50, Rotation2d.fromDegrees(-45.0)));
        for (int cycle = 0; cycle < 12 && scheduler.isScheduled(scheduledFollower); cycle++) {
            scheduler.run();
        }

        assertFalse(scheduler.isScheduled(scheduledFollower), "Follower should finish at the loaded path endpoint");
        assertEquals(0.0, robot.commandedSpeeds.vxMetersPerSecond, EPSILON);
        assertEquals(0.0, robot.commandedSpeeds.vyMetersPerSecond, EPSILON);
        assertEquals(0.0, robot.commandedSpeeds.omegaRadiansPerSecond, EPSILON);
    }

    private static FollowPath.Builder builder(MutableRobot robot) {
        return new FollowPath.Builder(
            new TestDriveSubsystem(),
            robot::getPose,
            robot::getSpeeds,
            robot::acceptSpeeds,
            new PIDController(2.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0),
            new PIDController(1.0, 0.0, 0.0)
        );
    }

    private void scheduleAndRunOnce(Command command) {
        scheduler.schedule(command);
        scheduler.run();
        assertTrue(scheduler.isScheduled(command));
    }

    private static Path loadRepresentativePath() throws URISyntaxException {
        return new Path(resourceAutosDirectory(), "behavior-parity");
    }

    private static File resourceAutosDirectory() throws URISyntaxException {
        URL resource = BehavioralCompatibilityTest.class.getResource("/behavior-parity/autos");
        assertNotNull(resource, "Representative autos test resource must be present");
        return new File(resource.toURI());
    }

    private static void assertPose(Pose2d pose, double x, double y, double degrees) {
        assertEquals(x, pose.getX(), EPSILON);
        assertEquals(y, pose.getY(), EPSILON);
        assertEquals(degrees, pose.getRotation().getDegrees(), EPSILON);
    }

    private static final class TestDriveSubsystem implements Subsystem {}

    private static final class MutableRobot {
        private Pose2d pose = new Pose2d();
        private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();

        private Pose2d getPose() {
            return pose;
        }

        private void setPose(Pose2d pose) {
            this.pose = pose;
        }

        private ChassisSpeeds getSpeeds() {
            return commandedSpeeds;
        }

        private void acceptSpeeds(ChassisSpeeds speeds) {
            commandedSpeeds = speeds;
        }
    }
}
