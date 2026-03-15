package frc.robot.lib.BLine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.PathElementConstraint;
import frc.robot.lib.BLine.Path.EventTrigger;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.RotationTargetConstraint;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.TranslationTargetConstraint;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * A WPILib Command that follows a {@link Path} using PID controllers for translation and rotation.
 * 
 * <p>This command drives the robot along a defined path by tracking translation targets sequentially
 * while simultaneously managing rotation targets. The command uses three PID controllers:
 * <ul>
 *   <li><b>Translation Controller:</b> Calculates command speed by minimizing total path distance remaining</li>
 *   <li><b>Rotation Controller:</b> Controls holonomic rotation toward the current rotation target</li>
 *   <li><b>Cross-Track Controller:</b> Minimizes deviation from the line between waypoints</li>
 * </ul>
 * 
 * <p>The path following algorithm works by:
 * <ol>
 *   <li>Calculating command robot speed via a PID controller minimizing total path distance remaining</li>
 *   <li>Determining velocity direction by pointing toward the current translation target</li>
 *   <li>Advancing to the next translation target when within the handoff radius of the current one</li>
 *   <li>Applying cross-track correction to stay on the line between waypoints</li>
 *   <li>Interpolating rotation based on progress between rotation targets</li>
 *   <li>Applying rate limiting via {@link ChassisRateLimiter} to respect constraints</li>
 * </ol>
 * 
 * <h2>Usage</h2>
 * <p>Use the {@link Builder} class to construct FollowPath commands:
 * <pre>{@code
 * FollowPath.Builder pathBuilder = new FollowPath.Builder(
 *     driveSubsystem,
 *     this::getPose,
 *     this::getRobotRelativeSpeeds,
 *     this::driveRobotRelative,
 *     new PIDController(5.0, 0, 0),  // translation
 *     new PIDController(3.0, 0, 0),  // rotation
 *     new PIDController(2.0, 0, 0)   // cross-track
 * ).withDefaultShouldFlip()
 *  .withPoseReset(this::resetPose);
 * 
 * // Then build commands for specific paths:
 * Command followAuto = pathBuilder.build(new Path("myPath"));
 * }</pre>
 * 
 * <h2>Logging</h2>
 * <p>The command supports optional logging via consumer functions. Set up logging callbacks using:
 * <ul>
 *   <li>{@link #setPoseLoggingConsumer(Consumer)} - Log pose data</li>
 *   <li>{@link #setDoubleLoggingConsumer(Consumer)} - Log numeric values</li>
 *   <li>{@link #setBooleanLoggingConsumer(Consumer)} - Log boolean states</li>
 *   <li>{@link #setTranslationListLoggingConsumer(Consumer)} - Log translation arrays</li>
 * </ul>
 * 
 * @see Path
 * @see Builder
 * @see ChassisRateLimiter
 */
public class FollowPath extends Command {
    private static final java.util.logging.Logger logger = java.util.logging.Logger.getLogger(FollowPath.class.getName());
    // Shared epsilon for all segment-length degeneracy checks.
    private static final double SEGMENT_EPSILON = 1e-6;
    // Epsilon for t-ratio comparisons to avoid floating-point edge jitter.
    private static final double T_RATIO_EPSILON = 1e-9;
    // Explicit sentinel for "no active rotation target selected".
    private static final int NO_ACTIVE_ROTATION_INDEX = -1;
    // Defaults to FPGA-backed time but is overrideable in tests for deterministic simulation.
    private static Supplier<Double> timestampSupplier = Timer::getTimestamp;
    private static Consumer<Pair<String, Pose2d>> poseLoggingConsumer = value -> {};
    private static Consumer<Pair<String, Translation2d[]>> translationListLoggingConsumer = value -> {};
    private static Consumer<Pair<String, Double>> doubleLoggingConsumer = value -> {};
    private static Consumer<Pair<String, Boolean>> booleanLoggingConsumer = value -> {};
    private static final Map<String, Runnable> eventTriggerRegistry = new HashMap<>();

    private static void logDouble(String key, double value) {
        doubleLoggingConsumer.accept(new Pair<>(key, value));
    }

    private static void logBoolean(String key, boolean value) {
        booleanLoggingConsumer.accept(new Pair<>(key, value));
    }

    private static void logPose(String key, Pose2d value) {
        poseLoggingConsumer.accept(new Pair<>(key, value));
    }

    /**
     * Registers an event trigger action by key.
     *
     * @param key The event trigger key referenced in JSON
     * @param action The action to execute when the trigger is reached
     */
    public static void registerEventTrigger(String key, Runnable action) {
        if (key == null || key.isEmpty() || action == null) {
            logger.warning("FollowPath: Ignoring invalid event trigger registration");
            return;
        }
        eventTriggerRegistry.put(key, action);
    }

    /**
     * Registers an event trigger action by key using a WPILib Command.
     *
     * @param key The event trigger key referenced in JSON
     * @param command The command to schedule when the trigger is reached
     */
    public static void registerEventTrigger(String key, Command command) {
        if (command == null) {
            logger.warning("FollowPath: Ignoring null command registration for key: " + key);
            return;
        }
        registerEventTrigger(key, () -> CommandScheduler.getInstance().schedule(command));
    }

    private final PIDController translationController;
    private final PIDController rotationController;
    private final PIDController crossTrackController;

    private void configureControllers() {
        translationController.setTolerance(path.getEndTranslationToleranceMeters());
        rotationController.setTolerance(Math.toRadians(path.getEndRotationToleranceDeg()));
        crossTrackController.setTolerance(path.getEndTranslationToleranceMeters());
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Sets the consumer for logging pose data during path following.
     * 
     * <p>The consumer receives pairs of (key, Pose2d) for various internal poses such as
     * closest points on path segments.
     * 
     * @param poseLoggingConsumer The consumer to receive pose logging data, or null to disable
     */
    public static void setPoseLoggingConsumer(Consumer<Pair<String, Pose2d>> poseLoggingConsumer) {
        if (poseLoggingConsumer == null) { return; }
        FollowPath.poseLoggingConsumer = poseLoggingConsumer;
    }

    /**
     * Sets the consumer for logging translation arrays during path following.
     * 
     * <p>The consumer receives pairs of (key, Translation2d[]) for data such as path waypoints
     * and robot position history.
     * 
     * @param translationListLoggingConsumer The consumer to receive translation list data, or null to disable
     */
    public static void setTranslationListLoggingConsumer(Consumer<Pair<String, Translation2d[]>> translationListLoggingConsumer) {
        if (translationListLoggingConsumer == null) { return; }
        FollowPath.translationListLoggingConsumer = translationListLoggingConsumer;
    }

    /**
     * Sets the consumer for logging boolean values during path following.
     * 
     * <p>The consumer receives pairs of (key, Boolean) for state flags such as completion status.
     * 
     * @param booleanLoggingConsumer The consumer to receive boolean logging data, or null to disable
     */
    public static void setBooleanLoggingConsumer(Consumer<Pair<String, Boolean>> booleanLoggingConsumer) {
        if (booleanLoggingConsumer == null) { return; }
        FollowPath.booleanLoggingConsumer = booleanLoggingConsumer;
    }

    /**
     * Sets the consumer for logging numeric values during path following.
     * 
     * <p>The consumer receives pairs of (key, Double) for various metrics such as remaining
     * distance, controller outputs, and target indices.
     * 
     * @param doubleLoggingConsumer The consumer to receive double logging data, or null to disable
     */
    public static void setDoubleLoggingConsumer(Consumer<Pair<String, Double>> doubleLoggingConsumer) {
        if (doubleLoggingConsumer == null) { return; }
        FollowPath.doubleLoggingConsumer = doubleLoggingConsumer;
    }

    /**
     * Overrides the time source used to compute loop dt.
     *
     * <p>Passing null restores the default WPILib {@link Timer#getTimestamp()} source.
     * This exists primarily to allow deterministic unit tests without HAL timing dependencies.
     */
    static void setTimestampSupplier(Supplier<Double> supplier) {
        timestampSupplier = supplier == null ? Timer::getTimestamp : supplier;
    }
    
    
    private final Path path;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier;
    private final Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer;
    private final Supplier<Boolean> shouldFlipPathSupplier;
    private final Supplier<Boolean> shouldMirrorPathSupplier;
    private final Consumer<Pose2d> poseResetConsumer;
    private final boolean useTRatioBasedTranslationHandoffs;
    
    private int rotationElementIndex = NO_ACTIVE_ROTATION_INDEX;
    private int translationElementIndex = 0;
    private int eventTriggerElementIndex = 0;

    private ChassisSpeeds lastSpeeds = new ChassisSpeeds();
    private double lastTimestamp = 0;
    private Pose2d pathInitStartPose = new Pose2d();
    private double previousRotationElementTargetRad = 0;   
    private int previousRotationElementIndex = 0;
    private Rotation2d currentRotationTargetRad = new Rotation2d();
    private double currentRotationTargetInitRad = 0;
    private List<Pair<PathElement, PathElementConstraint>> pathElementsWithConstraints = new ArrayList<>();

    private int logCounter = 0;
    private ArrayList<Translation2d> robotTranslations = new ArrayList<>();
    private double cachedRemainingDistance = 0.0;
    private final Set<Integer> firedEventTriggerIndices = new HashSet<>();
    private int firedEventTriggerCount = 0;

    // Snapshot of the currently tracked translation segment and robot progress on it.
    private record TranslationSegmentState(
        int startTranslationIndex,
        int endTranslationIndex,
        Translation2d startTranslation,
        Translation2d endTranslation,
        double segmentLength,
        double segmentProgress
    ) {
        /** @return true when this segment is effectively zero-length. */
        private boolean isDegenerate() {
            return segmentLength < SEGMENT_EPSILON;
        }
    }

    // Bounds for the translation segment that a rotation target belongs to.
    private record RotationSegmentBounds(
        int startTranslationIndex,
        int endTranslationIndex,
        Translation2d startTranslation,
        Translation2d endTranslation
    ) {}

    // Selected rotation target for this cycle plus the most recently completed target.
    private record RotationSelection(
        int activeRotationIndex,
        int previousRotationIndex
    ) {}

    /**
     * Builder class for constructing {@link FollowPath} commands with a fluent API.
     * 
     * <p>The Builder allows you to configure a path follower once with all the robot-specific
     * parameters, then build multiple commands for different paths. This avoids repeating
     * the same configuration for each path.
     *
     * <p><b>Important:</b> This builder is mutable and stateful. Optional settings configured
     * through {@code with...} methods persist for all subsequent {@link #build(Path)} calls
     * until you change them again. For example, once {@link #withPoseReset(Consumer)} is set,
     * later built commands will continue resetting pose unless you override it (for example,
     * with a no-op consumer).
     * 
     * <h2>Required Parameters</h2>
     * <p>The constructor requires:
     * <ul>
     *   <li>Drive subsystem - For command requirements</li>
     *   <li>Pose supplier - Returns current robot pose</li>
     *   <li>Robot-relative speeds supplier - Returns current chassis speeds</li>
     *   <li>Robot-relative speeds consumer - Accepts commanded chassis speeds</li>
     *   <li>Three PID controllers for translation, rotation, and cross-track correction</li>
     * </ul>
     * 
     * <h2>Optional Configuration</h2>
     * <ul>
     *   <li>{@link #withShouldFlip(Supplier)} - Custom alliance flip logic</li>
     *   <li>{@link #withDefaultShouldFlip()} - Use DriverStation alliance for flipping</li>
     *   <li>{@link #withShouldMirror(Supplier)} - Custom vertical mirror logic</li>
     *   <li>{@link #withPoseReset(Consumer)} - Reset odometry to path start pose</li>
     * </ul>
     * 
     * <h2>Example</h2>
     * <pre>{@code
     * FollowPath.Builder builder = new FollowPath.Builder(
     *     driveSubsystem,
     *     this::getPose,
     *     this::getSpeeds,
     *     this::drive,
     *     translationPID,
     *     rotationPID,
     *     crossTrackPID
     * ).withDefaultShouldFlip();
     * 
     * Command cmd = builder.build(myPath);
     * }</pre>
     */
    public static class Builder {
        private final Subsystem driveSubsystem;
        private final Supplier<Pose2d> poseSupplier;
        private final Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier;
        private final Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer;
        private final PIDController translationController;
        private final PIDController rotationController;
        private final PIDController crossTrackController;
        
        private Supplier<Boolean> shouldFlipPathSupplier = () -> false;
        private Supplier<Boolean> shouldMirrorPathSupplier = () -> false;
        private Consumer<Pose2d> poseResetConsumer = (pose) -> {};
        private boolean useTRatioBasedTranslationHandoffs = false;
        
        /**
         * Creates a new FollowPath Builder with the required configuration.
         * 
         * @param driveSubsystem The drive subsystem that the command will require
         * @param poseSupplier Supplier that returns the current robot pose in field coordinates
         * @param robotRelativeSpeedsSupplier Supplier that returns current robot-relative chassis speeds
         * @param robotRelativeSpeedsConsumer Consumer that accepts robot-relative chassis speeds to drive
         * @param translationController PID controller for calculating command speed by minimizing path distance remaining
         * @param rotationController PID controller for rotating toward rotation targets
         * @param crossTrackController PID controller for staying on the line between waypoints
         */
        public Builder(
            Subsystem driveSubsystem, 
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer,
            PIDController translationController,
            PIDController rotationController,
            PIDController crossTrackController
        ) {
            this.driveSubsystem = driveSubsystem;
            this.poseSupplier = poseSupplier;
            this.robotRelativeSpeedsSupplier = robotRelativeSpeedsSupplier;
            this.robotRelativeSpeedsConsumer = robotRelativeSpeedsConsumer;
            this.translationController = translationController;
            this.rotationController = rotationController;
            this.crossTrackController = crossTrackController;
        }

        /**
         * Configures a custom supplier to determine whether the path should be flipped.
         * 
         * <p>When the supplier returns true, the path will be flipped to the opposite alliance
         * side using {@link FlippingUtil} during command initialization.
         *
         * <p>This setting persists for future {@link #build(Path)} calls until changed.
         * 
         * @param shouldFlipPathSupplier Supplier returning true if the path should be flipped
         * @return This builder for chaining
         */
        public Builder withShouldFlip(Supplier<Boolean> shouldFlipPathSupplier) {
            this.shouldFlipPathSupplier = shouldFlipPathSupplier;
            return this;
        }

        /**
         * Configures the builder to use the default alliance-based path flipping.
         * 
         * <p>When enabled, paths will automatically be flipped when the robot is on the
         * red alliance, based on {@link edu.wpi.first.wpilibj.DriverStation#getAlliance()}.
         *
         * <p>This setting persists for future {@link #build(Path)} calls until changed.
         * 
         * @return This builder for chaining
         */
        public Builder withDefaultShouldFlip() {
            this.shouldFlipPathSupplier = FollowPath::shouldFlipPath;
            return this;
        }

        /**
         * Configures a custom supplier to determine whether the path should be mirrored vertically.
         * 
         * <p>When the supplier returns true, the path will be mirrored over the vertical
         * direction across the field width ({@code y -> fieldSizeY - y}) via {@link Path#mirror()}.
         *
         * <p>This setting persists for future {@link #build(Path)} calls until changed.
         * 
         * @param shouldMirrorPathSupplier Supplier returning true if the path should be mirrored vertically
         * @return This builder for chaining
         */
        public Builder withShouldMirror(Supplier<Boolean> shouldMirrorPathSupplier) {
            this.shouldMirrorPathSupplier = shouldMirrorPathSupplier;
            return this;
        }

        /**
         * Configures a consumer to reset the robot's pose at the start of path following.
         * 
         * <p>When set, the command will call this consumer with the path's starting pose
         * during initialization. This is useful for resetting odometry when starting autonomous
         * routines or when the robot is placed at a known location.
         *
         * <p>This setting persists for future {@link #build(Path)} calls until changed.
         * To disable pose reset on later commands when reusing the same builder, set a no-op
         * consumer such as {@code withPoseReset(pose -> {})}.
         * 
         * @param poseResetConsumer Consumer that resets the robot's pose estimate
         * @return This builder for chaining
         */
        public Builder withPoseReset(Consumer<Pose2d> poseResetConsumer) {
            this.poseResetConsumer = poseResetConsumer;
            return this;
        }

        /**
         * Enables or disables t_ratio-based translation handoffs.
         *
         * <p>When enabled, translation target handoffs occur based on projected
         * segment progress rather than raw distance, which can be more robust at
         * higher speeds on riskier paths. Defaults to false.
         *
         * <p>This setting persists for future {@link #build(Path)} calls until changed.
         *
         * @param enabled true to use t_ratio-based handoffs, false for radius-based
         * @return This builder for chaining
         */
        public Builder withTRatioBasedTranslationHandoffs(boolean enabled) {
            this.useTRatioBasedTranslationHandoffs = enabled;
            return this;
        }

        /**
         * Builds a FollowPath command for the specified path.
         * 
         * <p>The built command will use all the configuration from this builder. Each call
         * to build() creates an independent command that can be scheduled, using the builder's
         * current optional settings at the time of the call.
         * 
         * @param path The path to follow
         * @return A new FollowPath command configured for the given path
         * @throws IllegalArgumentException if any required controllers are null
         */
        public FollowPath build(Path path) {
            return new FollowPath(
                path,
                driveSubsystem,
                poseSupplier,
                robotRelativeSpeedsSupplier,
                robotRelativeSpeedsConsumer,
                shouldFlipPathSupplier,
                shouldMirrorPathSupplier,
                poseResetConsumer,
                useTRatioBasedTranslationHandoffs,
                translationController,
                rotationController,
                crossTrackController
            );
        }
    }

    /**
     * Determines if the path should be flipped based on the current alliance.
     * 
     * @return true if on the red alliance and the path should be flipped, false otherwise
     */
    private static boolean shouldFlipPath() {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
        }
        return false;
    }

    private FollowPath(
        Path path, 
        Subsystem driveSubsystem, 
        Supplier<Pose2d> poseSupplier, 
        Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
        Consumer<ChassisSpeeds> robotRelativeSpeedsConsumer,
        Supplier<Boolean> shouldFlipPathSupplier,
        Supplier<Boolean> shouldMirrorPathSupplier,
        Consumer<Pose2d> poseResetConsumer,
        boolean useTRatioBasedTranslationHandoffs,
        PIDController translationController, 
        PIDController rotationController,
        PIDController crossTrackController
    ) {
        if (translationController == null || rotationController == null || crossTrackController == null) {
            throw new IllegalArgumentException("Controllers must be provided and must not be null");
        }

        this.path = path.copy();
        this.poseSupplier = poseSupplier;
        this.robotRelativeSpeedsSupplier = robotRelativeSpeedsSupplier;
        this.robotRelativeSpeedsConsumer = robotRelativeSpeedsConsumer;
        this.shouldFlipPathSupplier = shouldFlipPathSupplier;
        this.shouldMirrorPathSupplier = shouldMirrorPathSupplier;
        this.poseResetConsumer = poseResetConsumer;
        this.useTRatioBasedTranslationHandoffs = useTRatioBasedTranslationHandoffs;
        this.translationController = translationController;
        this.rotationController = rotationController;
        this.crossTrackController = crossTrackController;
        
        configureControllers();
        
        addRequirements(driveSubsystem);
    }


    @Override
    public void initialize() {
        if (translationController == null || rotationController == null) {
            throw new IllegalArgumentException("Translation and rotation controllers must be provided and must not be null");
        }

        if (!path.isValid()) {
            logger.log(java.util.logging.Level.WARNING, "FollowPath: Path invalid - skipping initialization");
            return;
        }

        if (shouldFlipPathSupplier.get()) {
            path.flip();
        }
        if (shouldMirrorPathSupplier.get()) {
            path.mirror();
        }
        pathElementsWithConstraints = path.getPathElementsWithConstraintsNoWaypoints();

        // Resolve and apply start pose once so all segment-relative calculations share a stable origin.
        if (pathElementsWithConstraints.isEmpty()) {
            throw new IllegalStateException("Path must contain at least one element");
        }

        Pose2d startPose = path.getStartPose(poseSupplier.get().getRotation());
        poseResetConsumer.accept(startPose);

        // Reset traversal state for a fresh command run.
        rotationElementIndex = NO_ACTIVE_ROTATION_INDEX;
        translationElementIndex = 0;
        eventTriggerElementIndex = 0;
        firedEventTriggerIndices.clear();
        firedEventTriggerCount = 0;
        lastTimestamp = timestampSupplier.get();
        pathInitStartPose = poseSupplier.get();
        lastSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeedsSupplier.get(), pathInitStartPose.getRotation());
        previousRotationElementTargetRad = pathInitStartPose.getRotation().getRadians();
        previousRotationElementIndex = -1;
        currentRotationTargetInitRad = pathInitStartPose.getRotation().getRadians();
        rotationController.reset();
        translationController.reset();
        configureControllers();

        ArrayList<Translation2d> pathTranslations = new ArrayList<>();
        robotTranslations.clear();
        logCounter = 0;
        for (int i = 0; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                pathTranslations.add(((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation());
            }
        }
        logBoolean("FollowPath/useTRatioBasedTranslationHandoffs", useTRatioBasedTranslationHandoffs);
        translationListLoggingConsumer.accept(new Pair<>("FollowPath/pathTranslations", pathTranslations.toArray(Translation2d[]::new)));
    }

    @Override
    public void execute() {
        if (!path.isValid()) {
            logger.log(java.util.logging.Level.WARNING, "FollowPath: Path invalid - skipping execution");
            stopCommandedMotion();
            return;
        }
        double now = timestampSupplier.get();
        double dt = now - lastTimestamp;
        lastTimestamp = now;
        logDouble("FollowPath/dtSeconds", dt);

        Pose2d currentPose = poseSupplier.get();

        // Phase 1: verify translation cursor integrity before doing any control math.
        if (translationElementIndex >= pathElementsWithConstraints.size()) {
            logger.warning("FollowPath: Translation element index out of bounds");
            stopCommandedMotion();
            return;
        }
        if (!isTranslationTargetAt(translationElementIndex)) {
            logger.warning("FollowPath: Expected TranslationTarget at index " + translationElementIndex);
            stopCommandedMotion();
            return;
        }

        // Phase 2: advance translation target(s). This may skip multiple targets in one cycle.
        int previousTranslationIndex = translationElementIndex;
        advanceTranslationTargets(currentPose);
        boolean translationHandoffOccurred = translationElementIndex != previousTranslationIndex;
        logBoolean("FollowPath/translationHandoffOccurred", translationHandoffOccurred);
        if (translationHandoffOccurred) {
            logDouble("FollowPath/translationHandoffFromIndex", (double) previousTranslationIndex);
            logDouble("FollowPath/translationHandoffToIndex", (double) translationElementIndex);
        }
        if (translationElementIndex >= pathElementsWithConstraints.size() || !isTranslationTargetAt(translationElementIndex)) {
            logger.warning("FollowPath: Invalid translation target after handoff at index " + translationElementIndex);
            stopCommandedMotion();
            return;
        }

        TranslationSegmentState currentSegment = getCurrentTranslationSegmentState(currentPose);
        logDouble("FollowPath/currentSegmentLengthMeters", currentSegment.segmentLength());
        logDouble("FollowPath/currentSegmentProgress", currentSegment.segmentProgress());
        logBoolean("FollowPath/currentSegmentDegenerate", currentSegment.isDegenerate());

        // Phase 3: choose rotation target for this cycle.
        // If no target exists on this segment, selection intentionally looks ahead to future segments.
        int lastRotationElementIndex = rotationElementIndex;
        RotationSelection rotationSelection = selectRotationTarget(currentSegment);
        rotationElementIndex = rotationSelection.activeRotationIndex() >= 0
            ? rotationSelection.activeRotationIndex()
            : NO_ACTIVE_ROTATION_INDEX;

        // Keep interpolation anchor synced to the most recently completed rotation target.
        if (rotationSelection.previousRotationIndex() >= 0 &&
            rotationSelection.previousRotationIndex() != previousRotationElementIndex &&
            pathElementsWithConstraints.get(rotationSelection.previousRotationIndex()).getFirst() instanceof RotationTarget) {
            previousRotationElementTargetRad = ((RotationTarget) pathElementsWithConstraints.get(rotationSelection.previousRotationIndex()).getFirst()).rotation().getRadians();
            previousRotationElementIndex = rotationSelection.previousRotationIndex();
            currentRotationTargetInitRad = currentPose.getRotation().getRadians();
        }
        if (lastRotationElementIndex != rotationElementIndex) {
            logDouble("FollowPath/rotationElementIndex", (double) rotationElementIndex);
        }
        logBoolean("FollowPath/rotationHasActiveTarget", rotationSelection.activeRotationIndex() >= 0);
        logDouble("FollowPath/rotationPreviousElementIndex", (double) rotationSelection.previousRotationIndex());

        // Event triggers use the same segment/t-ratio semantics as rotation targets.
        processEventTriggers(currentPose);

        // Phase 4: compute translational command vector.
        Translation2d targetTranslation = isTranslationTargetAt(translationElementIndex)
            ? ((TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst()).translation()
            : currentPose.getTranslation();
        double remainingDistance = calculateRemainingPathDistance();
        double angleToTarget = Math.atan2(
            targetTranslation.getY() - currentPose.getTranslation().getY(),
            targetTranslation.getX() - currentPose.getTranslation().getX()
        );

        if (!(pathElementsWithConstraints.get(translationElementIndex).getSecond() instanceof TranslationTargetConstraint)) {
            logger.warning("FollowPath: Expected TranslationTargetConstraint at index " + translationElementIndex);
            stopCommandedMotion();
            return;
        }
        TranslationTargetConstraint translationConstraint = (TranslationTargetConstraint) pathElementsWithConstraints.get(translationElementIndex).getSecond();

        // Clamp translation controller output as to not overpower the crossTrackController output during the velo accel limiting phase
        double translationControllerOutput = MathUtil.clamp(-translationController.calculate(remainingDistance, 0), -translationConstraint.maxVelocityMetersPerSec(), translationConstraint.maxVelocityMetersPerSec());
        logDouble("FollowPath/translationControllerOutput", translationControllerOutput);
        
        // Cache the remaining distance for logging
        cachedRemainingDistance = remainingDistance;
        double vx = translationControllerOutput * Math.cos(angleToTarget);
        double vy = translationControllerOutput * Math.sin(angleToTarget);

        double crossTrackError = calculateCrossTrackError();

        // dont clamp cross track controller as users may prefer to tune their controller to be hyper response to cross track
        double crossTrackControllerOutput = -crossTrackController.calculate(crossTrackError, 0);
        logDouble("FollowPath/crossTrackControllerOutput", crossTrackControllerOutput);

        // Rotate the cross-track correction into field frame and add it to translation command.
        vx += crossTrackControllerOutput * Math.cos(angleToTarget - Math.PI / 2);
        vy += crossTrackControllerOutput * Math.sin(angleToTarget - Math.PI / 2);

        double targetRotationRad;
        RotationTargetConstraint rotationConstraint;

        // Phase 5: compute rotational command setpoint.
        if (rotationSelection.activeRotationIndex() >= 0 && isRotationTargetAt(rotationSelection.activeRotationIndex())) {

            RotationTarget currentRotationTarget = (RotationTarget) pathElementsWithConstraints.get(rotationSelection.activeRotationIndex()).getFirst();
            if (!(pathElementsWithConstraints.get(rotationSelection.activeRotationIndex()).getSecond() instanceof RotationTargetConstraint)) {
                logger.warning("FollowPath: Expected RotationTargetConstraint at index " + rotationElementIndex);
                stopCommandedMotion();
                return;
            }
            rotationConstraint = (RotationTargetConstraint) pathElementsWithConstraints.get(rotationSelection.activeRotationIndex()).getSecond();
            currentRotationTargetRad = currentRotationTarget.rotation();

            if (currentRotationTarget.profiledRotation()) {
                // Calculate rotation progress using t-ratio projection between rotation target positions.
                Translation2d rotationStart = rotationSelection.previousRotationIndex() < 0
                    ? pathInitStartPose.getTranslation()
                    : calculateRotationTargetTranslation(rotationSelection.previousRotationIndex());
                Translation2d rotationEnd = calculateRotationTargetTranslation(rotationSelection.activeRotationIndex());
                double rotationSegmentLength = rotationStart.getDistance(rotationEnd);
                // Degenerate rotation segment means interpolation is complete immediately.
                double segmentProgress = rotationSegmentLength < SEGMENT_EPSILON
                    ? 1.0
                    : calculateSegmentProjectionT(rotationStart, rotationEnd, currentPose.getTranslation());

                // Snap rotation to complete when within end-translation tolerance of the target.
                // This prevents undershooting the final rotation when translation stops early.
                double endTranslationTolerance = path.getEndTranslationToleranceMeters();
                if (rotationSegmentLength > SEGMENT_EPSILON && endTranslationTolerance > 0) {
                    double effectiveTolerance = Math.min(endTranslationTolerance, rotationSegmentLength);
                    double toleranceThreshold = 1.0 - (effectiveTolerance / rotationSegmentLength);
                    if (segmentProgress >= toleranceThreshold) {
                        segmentProgress = 1.0;
                    }
                }

                logDouble("FollowPath/segmentProgress", segmentProgress);

                // Calculate the shortest angular path from current robot rotation to target
                double endRotation = currentRotationTarget.rotation().getRadians();
                // Normalize the rotation difference to [-π, π] to take shortest path
                double rotationDifference = MathUtil.angleModulus(endRotation - previousRotationElementTargetRad);

                // Interpolate along the shortest path
                targetRotationRad = previousRotationElementTargetRad + segmentProgress * rotationDifference;
            } else {
                targetRotationRad = MathUtil.angleModulus(currentRotationTarget.rotation().getRadians());
            }

        } else {
            // No remaining rotation targets: hold the most recently completed target heading.
            targetRotationRad = previousRotationElementTargetRad;
            currentRotationTargetRad = new Rotation2d(targetRotationRad);
            rotationConstraint = new RotationTargetConstraint(
                    path.getDefaultGlobalConstraints().getMaxVelocityDegPerSec(), 
                    path.getDefaultGlobalConstraints().getMaxAccelerationDegPerSec2()
                );
        }

        targetRotationRad = MathUtil.angleModulus(targetRotationRad);
        double omega = rotationController.calculate(currentPose.getRotation().getRadians(), targetRotationRad);

        // Phase 6: apply acceleration/velocity limiting and output final command.
        ChassisSpeeds targetSpeeds = new ChassisSpeeds(vx, vy, omega);
        targetSpeeds = ChassisRateLimiter.limit(
            targetSpeeds, 
            lastSpeeds, 
            dt, 
            translationConstraint.maxAccelerationMetersPerSec2(),
            Math.toRadians(rotationConstraint.maxAccelerationDegPerSec2()),
            translationConstraint.maxVelocityMetersPerSec(),
            Math.toRadians(rotationConstraint.maxVelocityDegPerSec())
        );

        robotRelativeSpeedsConsumer.accept(ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, currentPose.getRotation()));

        lastSpeeds = targetSpeeds;

        if (logCounter++ % 3 == 0) {
            robotTranslations.add(currentPose.getTranslation());

            // Limit memory usage by keeping only the most recent points
            if (robotTranslations.size() > 300) {
                // Remove oldest entries to keep only the last 300 points
                robotTranslations.subList(0, robotTranslations.size() - 250).clear();
            }

            translationListLoggingConsumer.accept(new Pair<>("FollowPath/robotTranslations", robotTranslations.toArray(Translation2d[]::new)));
        }
        
        logDouble("FollowPath/remainingPathDistanceMeters", cachedRemainingDistance);
        logDouble("FollowPath/translationElementIndex", (double) translationElementIndex);
        logDouble("FollowPath/rotationElementIndex", (double) rotationElementIndex);
        logDouble("FollowPath/targetRotationDeg", Math.toDegrees(targetRotationRad));
        logDouble("FollowPath/rotationControllerOutput", omega);
        logDouble("FollowPath/rotationErrorDeg", Math.toDegrees(currentRotationTargetRad.minus(currentPose.getRotation()).getRadians()));
        logDouble("FollowPath/currentRotationTargetInitRad", currentRotationTargetInitRad);
        logDouble("FollowPath/eventTriggerElementIndex", (double) eventTriggerElementIndex);
        logDouble("FollowPath/eventTriggersFiredCount", (double) firedEventTriggerCount);
    }

    /**
     * Forces commanded chassis motion to zero and resets internal speed history.
     *
     * <p>Used on defensive early exits to avoid leaving stale nonzero velocity commands latched.
     */
    private void stopCommandedMotion() {
        ChassisSpeeds zeroSpeeds = new ChassisSpeeds();
        robotRelativeSpeedsConsumer.accept(zeroSpeeds);
        lastSpeeds = zeroSpeeds;
    }

    private boolean isTranslationTargetAt(int index) {
        return index >= 0 &&
            index < pathElementsWithConstraints.size() &&
            pathElementsWithConstraints.get(index).getFirst() instanceof TranslationTarget;
    }

    private boolean isRotationTargetAt(int index) {
        return index >= 0 &&
            index < pathElementsWithConstraints.size() &&
            pathElementsWithConstraints.get(index).getFirst() instanceof RotationTarget;
    }

    /**
     * Advances translation targets until the current target is no longer handoff-eligible.
     *
     * <p>This intentionally supports "draining" through multiple targets in one cycle,
     * which avoids one-cycle stalls on chains of tiny/degenerate segments.
     */
    private void advanceTranslationTargets(Pose2d currentPose) {
        while (true) {
            if (translationElementIndex >= pathElementsWithConstraints.size() ||
                !(pathElementsWithConstraints.get(translationElementIndex).getFirst() instanceof TranslationTarget)) {
                return;
            }

            int nextTranslationIndex = findNextTranslationTargetIndex(translationElementIndex + 1);
            if (nextTranslationIndex < 0) {
                return;
            }

            TranslationTarget currentTranslationTarget = (TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst();
            double handoffRadius = currentTranslationTarget.intermediateHandoffRadiusMeters()
                .orElse(path.getDefaultGlobalConstraints().getIntermediateHandoffRadiusMeters());

            TranslationSegmentState currentSegment = getCurrentTranslationSegmentState(currentPose);
            if (!shouldHandoffTranslationTarget(currentPose, currentTranslationTarget, currentSegment, handoffRadius)) {
                return;
            }

            translationElementIndex = nextTranslationIndex;
        }
    }

    /**
     * Determines whether the current translation target should hand off to the next target.
     *
     * <p>Degenerate segments always hand off immediately to avoid zero-length deadlocks.
     */
    private boolean shouldHandoffTranslationTarget(
        Pose2d currentPose,
        TranslationTarget currentTranslationTarget,
        TranslationSegmentState currentSegment,
        double handoffRadius
    ) {
        double distanceToTarget = currentPose.getTranslation().getDistance(currentTranslationTarget.translation());

        if (currentSegment.isDegenerate()) {
            return true;
        }

        if (!useTRatioBasedTranslationHandoffs) {
            return distanceToTarget <= handoffRadius;
        }

        double handoffThreshold = 1.0 - (handoffRadius / currentSegment.segmentLength());
        handoffThreshold = Math.max(0.0, Math.min(1.0, handoffThreshold));
        return currentSegment.segmentProgress() >= handoffThreshold
            || distanceToTarget <= handoffRadius;
    }

    /**
     * Computes segment start/end geometry for the current translation cursor.
     *
     * <p>If the cursor is invalid, returns a degenerate segment at the robot position
     * so callers can handle the error path uniformly.
     */
    private TranslationSegmentState getCurrentTranslationSegmentState(Pose2d currentPose) {
        if (translationElementIndex < 0 || translationElementIndex >= pathElementsWithConstraints.size() ||
            !(pathElementsWithConstraints.get(translationElementIndex).getFirst() instanceof TranslationTarget)) {
            Translation2d currentTranslation = currentPose.getTranslation();
            return new TranslationSegmentState(
                -1,
                translationElementIndex,
                currentTranslation,
                currentTranslation,
                0.0,
                1.0
            );
        }

        int startTranslationIndex = findPreviousTranslationTargetIndex(translationElementIndex - 1);
        Translation2d startTranslation = startTranslationIndex >= 0
            ? getTranslationAtIndex(startTranslationIndex)
            : pathInitStartPose.getTranslation();
        Translation2d endTranslation = getTranslationAtIndex(translationElementIndex);
        double segmentLength = startTranslation.getDistance(endTranslation);
        double segmentProgress = segmentLength < SEGMENT_EPSILON
            ? 1.0
            : calculateSegmentProjectionT(startTranslation, endTranslation, currentPose.getTranslation());

        return new TranslationSegmentState(
            startTranslationIndex,
            translationElementIndex,
            startTranslation,
            endTranslation,
            segmentLength,
            segmentProgress
        );
    }

    /**
     * Selects the active rotation target for this cycle.
     *
     * <p>Selection rule:
     * 1) If current segment has candidates, choose by segment progress.
     * 2) If current segment is degenerate, choose highest t_ratio candidate.
     * 3) If no current-segment candidate exists, look ahead to the next segment with a target.
     */
    private RotationSelection selectRotationTarget(TranslationSegmentState currentSegment) {
        int previousRotationIndex = -1;
        int activeRotationIndex = -1;
        double maxTRatioOnCurrentSegment = getMaxTRatioOnSegment(currentSegment.endTranslationIndex());

        for (int i = 0; i < pathElementsWithConstraints.size(); i++) {
            if (!(pathElementsWithConstraints.get(i).getFirst() instanceof RotationTarget)) {
                continue;
            }

            RotationSegmentBounds bounds = getRotationSegmentBounds(i);
            if (bounds == null) {
                previousRotationIndex = i;
                continue;
            }

            if (bounds.endTranslationIndex() < currentSegment.endTranslationIndex()) {
                previousRotationIndex = i;
                continue;
            }

            if (bounds.endTranslationIndex() > currentSegment.endTranslationIndex()) {
                activeRotationIndex = i;
                break;
            }

            RotationTarget rotationTarget = (RotationTarget) pathElementsWithConstraints.get(i).getFirst();
            double targetTRatio = clampTRatio(rotationTarget.t_ratio());
            if (currentSegment.isDegenerate()) {
                if (targetTRatio + T_RATIO_EPSILON < maxTRatioOnCurrentSegment) {
                    previousRotationIndex = i;
                    continue;
                }
                activeRotationIndex = i;
                break;
            }

            if (targetTRatio <= currentSegment.segmentProgress() + T_RATIO_EPSILON) {
                previousRotationIndex = i;
                continue;
            }

            activeRotationIndex = i;
            break;
        }

        return new RotationSelection(activeRotationIndex, previousRotationIndex);
    }

    /**
     * Finds the largest clamped t_ratio among rotation targets on a given segment endpoint.
     */
    private double getMaxTRatioOnSegment(int segmentEndTranslationIndex) {
        double maxTRatio = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < pathElementsWithConstraints.size(); i++) {
            if (!(pathElementsWithConstraints.get(i).getFirst() instanceof RotationTarget)) {
                continue;
            }
            RotationSegmentBounds bounds = getRotationSegmentBounds(i);
            if (bounds == null || bounds.endTranslationIndex() != segmentEndTranslationIndex) {
                continue;
            }
            double clampedTRatio = clampTRatio(((RotationTarget) pathElementsWithConstraints.get(i).getFirst()).t_ratio());
            maxTRatio = Math.max(maxTRatio, clampedTRatio);
        }
        return maxTRatio;
    }

    /** Clamps t_ratio into [0, 1]. */
    private double clampTRatio(double tRatio) {
        return Math.max(0.0, Math.min(1.0, tRatio));
    }

    /**
     * Finds the next translation target index at or after {@code startIndex}.
     *
     * @return translation index or -1 if none exists
     */
    private int findNextTranslationTargetIndex(int startIndex) {
        for (int i = Math.max(startIndex, 0); i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Finds the previous translation target index at or before {@code startIndex}.
     *
     * @return translation index or -1 if none exists
     */
    private int findPreviousTranslationTargetIndex(int startIndex) {
        for (int i = Math.min(startIndex, pathElementsWithConstraints.size() - 1); i >= 0; i--) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Returns translation at a known translation target index, or start pose translation if invalid.
     */
    private Translation2d getTranslationAtIndex(int translationIndex) {
        if (translationIndex >= 0 && translationIndex < pathElementsWithConstraints.size() &&
            pathElementsWithConstraints.get(translationIndex).getFirst() instanceof TranslationTarget) {
            return ((TranslationTarget) pathElementsWithConstraints.get(translationIndex).getFirst()).translation();
        }
        return pathInitStartPose.getTranslation();
    }
    
    /**
     * Calculates the total remaining path distance from the robot's current position.
     * 
     * <p>This is used by the translation controller to calculate command speed.
     * Sums the distances from the current position through all remaining translation targets.
     * 
     * @return The remaining path distance in meters
     */
    private double calculateRemainingPathDistance() {
        Translation2d previousTranslation = poseSupplier.get().getTranslation();
        double remainingDistance = 0;
        for (int i = translationElementIndex; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                remainingDistance += previousTranslation.getDistance(
                    ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation()
                );
                previousTranslation = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
            }
        }
        return remainingDistance;
    }

    /**
     * Calculates the signed cross-track error from the robot to the line between waypoints.
     * 
     * <p>Positive values indicate the robot is to the right of the path, negative values
     * indicate the robot is to the left of the path.
     * 
     * @return The signed cross-track error in meters
     */
    private double calculateCrossTrackError() {
        Translation2d targetTranslation = ((TranslationTarget) pathElementsWithConstraints.get(translationElementIndex).getFirst()).translation();
        Translation2d prevTranslation = getCurrentTranslationSegmentStart();

        Pose2d currentPose = poseSupplier.get();
        Translation2d robotPosition = currentPose.getTranslation();

        // Find closest point on the segment using shared projection utility
        Translation2d closestPoint = calculateProjectedPointOnSegment(prevTranslation, targetTranslation, robotPosition);

        // Calculate signed cross-track error
        // Positive = right of path, Negative = left of path
        double pathVectorX = targetTranslation.getX() - prevTranslation.getX();
        double pathVectorY = targetTranslation.getY() - prevTranslation.getY();
        double robotVectorX = robotPosition.getX() - prevTranslation.getX();
        double robotVectorY = robotPosition.getY() - prevTranslation.getY();

        // Cross product to determine side: positive = left, negative = right
        double crossProduct = pathVectorX * robotVectorY - pathVectorY * robotVectorX;

        // Return signed distance (positive = right of path, negative = left of path)
        double signedError = robotPosition.getDistance(closestPoint);
        if (crossProduct < 0) {
            signedError = -signedError; // Left of path = negative
        }
        // Right of path = positive (crossProduct > 0), so no change needed

        logPose("FollowPath/closestPoint", new Pose2d(closestPoint, currentPose.getRotation()));
        logDouble("FollowPath/crossTrackError", signedError);

        return signedError;
    }

    /**
     * Calculates the clamped projection ratio of a point onto a segment.
     *
     * @param segmentStart The start of the segment
     * @param segmentEnd The end of the segment
     * @param point The point to project
     * @return Projection ratio along the segment in [0, 1]
     */
    private double calculateSegmentProjectionT(
        Translation2d segmentStart,
        Translation2d segmentEnd,
        Translation2d point
    ) {
        double dx = segmentEnd.getX() - segmentStart.getX();
        double dy = segmentEnd.getY() - segmentStart.getY();
        double segmentLengthSquared = dx * dx + dy * dy;
        if (segmentLengthSquared < SEGMENT_EPSILON) {
            return 0.0;
        }

        double dxPoint = point.getX() - segmentStart.getX();
        double dyPoint = point.getY() - segmentStart.getY();
        double t = (dxPoint * dx + dyPoint * dy) / segmentLengthSquared;
        return Math.max(0.0, Math.min(1.0, t));
    }

    /**
     * Calculates the projected point on a segment for a given position.
     *
     * @param segmentStart The start of the segment
     * @param segmentEnd The end of the segment
     * @param point The point to project
     * @return The projected point on the segment
     */
    private Translation2d calculateProjectedPointOnSegment(
        Translation2d segmentStart,
        Translation2d segmentEnd,
        Translation2d point
    ) {
        double t = calculateSegmentProjectionT(segmentStart, segmentEnd, point);
        double dx = segmentEnd.getX() - segmentStart.getX();
        double dy = segmentEnd.getY() - segmentStart.getY();
        return new Translation2d(
            segmentStart.getX() + t * dx,
            segmentStart.getY() + t * dy
        );
    }

    /**
     * Gets the start point for the current translation segment.
     *
     * <p>This walks backward from the current translation element to find the
     * previous translation target. If none exists, it falls back to the path
     * initialization pose. This keeps cross-track calculations stable when
     * translation targets switch.
     *
     * @return The start translation for the current segment
     */
    private Translation2d getCurrentTranslationSegmentStart() {
        int previousTranslationIndex = findPreviousTranslationTargetIndex(translationElementIndex - 1);
        return previousTranslationIndex >= 0
            ? getTranslationAtIndex(previousTranslationIndex)
            : pathInitStartPose.getTranslation();
    }

    /**
     * Calculates the field position where a rotation target should be achieved.
     * 
     * <p>Rotation targets are interpolated between translation targets using their t_ratio.
     * 
     * @param index The index of the rotation target in the path elements list
     * @return The translation where this rotation should be achieved
     */
    private Translation2d calculateRotationTargetTranslation(int index) {
        if (index < 0 || index >= pathElementsWithConstraints.size() ||
            !(pathElementsWithConstraints.get(index).getFirst() instanceof RotationTarget)) {
            logger.warning("FollowPath: Invalid rotation target index: " + index);
            return new Translation2d();
        }

        RotationSegmentBounds bounds = getRotationSegmentBounds(index);
        if (bounds == null) {
            logger.warning("FollowPath: Missing translation bounds for rotation target at index " + index);
            return new Translation2d();
        }

        RotationTarget rotationTarget = (RotationTarget) pathElementsWithConstraints.get(index).getFirst();
        double tRatio = clampTRatio(rotationTarget.t_ratio());
        Translation2d pointOnSegment = new Translation2d(
            bounds.startTranslation().getX() + (bounds.endTranslation().getX() - bounds.startTranslation().getX()) * tRatio,
            bounds.startTranslation().getY() + (bounds.endTranslation().getY() - bounds.startTranslation().getY()) * tRatio
        );
        logPose("FollowPath/rotationTargetPose", new Pose2d(pointOnSegment, rotationTarget.rotation()));
        return pointOnSegment;
    }

    /**
     * Resolves the translation-segment bounds that contain a rotation target.
     *
     * <p>The "end translation index" identifies which translation segment the target belongs to,
     * which is used by selection logic to compare against current translation progress.
     */
    private RotationSegmentBounds getRotationSegmentBounds(int rotationIndex) {
        if (rotationIndex < 0 || rotationIndex >= pathElementsWithConstraints.size() ||
            !(pathElementsWithConstraints.get(rotationIndex).getFirst() instanceof RotationTarget)) {
            return null;
        }

        int startTranslationIndex = findPreviousTranslationTargetIndex(rotationIndex - 1);
        int endTranslationIndex = findNextTranslationTargetIndex(rotationIndex + 1);
        if (endTranslationIndex < 0) {
            return null;
        }

        Translation2d startTranslation = startTranslationIndex >= 0
            ? getTranslationAtIndex(startTranslationIndex)
            : pathInitStartPose.getTranslation();
        Translation2d endTranslation = getTranslationAtIndex(endTranslationIndex);
        return new RotationSegmentBounds(
            startTranslationIndex,
            endTranslationIndex,
            startTranslation,
            endTranslation
        );
    }

    /**
     * Processes event triggers in path order until the next trigger is not yet reached.
     */
    private void processEventTriggers(Pose2d currentPose) {
        while (eventTriggerElementIndex < pathElementsWithConstraints.size()) {
            PathElement element = pathElementsWithConstraints.get(eventTriggerElementIndex).getFirst();
            if (!(element instanceof EventTrigger)) {
                eventTriggerElementIndex++;
                continue;
            }
            if (firedEventTriggerIndices.contains(eventTriggerElementIndex)) {
                eventTriggerElementIndex++;
                continue;
            }
            if (!isEventTriggerTRatioReached(eventTriggerElementIndex, currentPose)) {
                break;
            }
            EventTrigger trigger = (EventTrigger) element;
            Runnable action = eventTriggerRegistry.get(trigger.libKey());
            if (action != null) {
                action.run();
            } else {
                logger.warning("FollowPath: Unregistered event trigger key: " + trigger.libKey());
            }
            firedEventTriggerIndices.add(eventTriggerElementIndex);
            firedEventTriggerCount++;
            eventTriggerElementIndex++;
        }
    }

    /**
     * Returns true when the trigger's t_ratio has been reached on its owning segment.
     *
     * <p>Degenerate event segments are treated as immediately reached.
     */
    private boolean isEventTriggerTRatioReached(int eventIndex, Pose2d currentPose) {
        if (eventIndex >= pathElementsWithConstraints.size() ||
            !(pathElementsWithConstraints.get(eventIndex).getFirst() instanceof EventTrigger)) {
            return false;
        }
        if (isEventTriggerNextSegment(eventIndex)) { return false; }
        if (isEventTriggerPreviousSegment(eventIndex)) { return true; }

        Translation2d translationA = null;
        Translation2d translationB = null;
        for (int i = eventIndex - 1; i >= 0; i--) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                translationA = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                break;
            }
        }
        for (int i = eventIndex + 1; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                translationB = ((TranslationTarget) pathElementsWithConstraints.get(i).getFirst()).translation();
                break;
            }
        }
        if (translationA == null || translationB == null) {
            logger.warning("FollowPath: Missing translation bounds for event trigger at index " + eventIndex);
            return false;
        }

        double segmentLength = translationA.getDistance(translationB);
        if (segmentLength < SEGMENT_EPSILON) {
            return true;
        }

        double segmentProgress = calculateSegmentProjectionT(
            translationA,
            translationB,
            currentPose.getTranslation()
        );

        double targetTRatio = ((EventTrigger) pathElementsWithConstraints.get(eventIndex).getFirst()).t_ratio();
        return segmentProgress >= targetTRatio;
    }

    private boolean isEventTriggerPreviousSegment(int eventIndex) {
        if (eventIndex > translationElementIndex) { return false; }
        for (int i = eventIndex; i < translationElementIndex; i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                return true;
            }
        }
        return false;
    }

    private boolean isEventTriggerNextSegment(int eventIndex) {
        return eventIndex > translationElementIndex;
    }

    @Override
    public boolean isFinished() { // TODO add final velocity tolerance
        if (!path.isValid()) {
            logger.log(java.util.logging.Level.WARNING, "FollowPath: Path invalid - finishing early");
            return true;
        }

        // Completion requires both translation and rotation traversal to be on their final targets.
        boolean isLastRotationElement = rotationElementIndex == NO_ACTIVE_ROTATION_INDEX;
        if (!isLastRotationElement) {
            isLastRotationElement = true;
            for (int i = rotationElementIndex + 1; i < pathElementsWithConstraints.size(); i++) {
                if (pathElementsWithConstraints.get(i).getFirst() instanceof RotationTarget) {
                    isLastRotationElement = false;
                    break;
                }
            }
        }
        boolean isLastTranslationElement = true;
        for (int i = translationElementIndex+1; i < pathElementsWithConstraints.size(); i++) {
            if (pathElementsWithConstraints.get(i).getFirst() instanceof TranslationTarget) {
                isLastTranslationElement = false;
                break;
            }
        }
        boolean translationAtSetpoint = translationController.atSetpoint();
        boolean rotationAtSetpoint = Math.abs(currentRotationTargetRad.minus(poseSupplier.get().getRotation()).getRadians()) < Math.toRadians(path.getEndRotationToleranceDeg());
        boolean finished = 
            isLastRotationElement && isLastTranslationElement && 
            translationAtSetpoint &&
            rotationAtSetpoint;

        logBoolean("FollowPath/finished", finished);
        logBoolean("FollowPath/finishedIsLastRotationElement", isLastRotationElement);
        logBoolean("FollowPath/finishedIsLastTranslationElement", isLastTranslationElement);
        logBoolean("FollowPath/finishedTranslationAtSetpoint", translationAtSetpoint);
        logBoolean("FollowPath/finishedRotationAtSetpoint", rotationAtSetpoint);
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        stopCommandedMotion();
    }

    /**
     * Gets the current rotation element index in the path.
     * 
     * <p>This index represents the current rotation target being tracked, where both
     * waypoint rotations and standalone rotation targets are counted together.
     * 
     * @return The current rotation element index (0-based), or -1 when no active rotation target remains
     */
    public int getCurrentRotationElementIndex() {
        return rotationElementIndex;
    }

    /**
     * Gets the current translation element index in the path.
     * 
     * <p>This index represents the current translation target being tracked, where both
     * waypoint translations and standalone translation targets are counted together.
     * 
     * @return The current translation element index (0-based)
     */
    public int getCurrentTranslationElementIndex() {
        return translationElementIndex;
    }

    /**
     * Gets the estimated remaining path distance from the robot's current position.
     *
     * <p>This value is computed live from the command's current traversal cursor and
     * translation targets. It mirrors the distance basis used by the translation controller
     * during execution.
     *
     * <p>Returns {@code 0.0} when the command is not in a valid traversal state
     * (for example, invalid path or uninitialized/invalid translation cursor).
     *
     * @return Remaining path distance in meters
     */
    public double getRemainingPathDistanceMeters() {
        if (!path.isValid() ||
            pathElementsWithConstraints.isEmpty() ||
            !isTranslationTargetAt(translationElementIndex)) {
            return 0.0;
        }
        return calculateRemainingPathDistance();
    }

}
