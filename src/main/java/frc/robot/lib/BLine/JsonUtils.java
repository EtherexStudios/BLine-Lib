package frc.robot.lib.BLine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.EventTrigger;
import frc.robot.lib.BLine.Path.RotationTarget;
import frc.robot.lib.BLine.Path.TranslationTarget;
import frc.robot.lib.BLine.Path.Waypoint;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Consumer;

/**
 * Utility class for loading and parsing path data from JSON files.
 * 
 * <p>This class provides methods to load {@link Path} objects from JSON files stored in the
 * robot's deploy directory. It supports loading individual paths as well as global constraint
 * configurations.
 * 
 * <h2>File Structure</h2>
 * <p>The expected directory structure under the deploy directory is:
 * <pre>
 * deploy/
 *   autos/
 *     config.json          (global constraints configuration)
 *     paths/
 *       myPath.json        (individual path files)
 *       otherPath.json
 * </pre>
 * 
 * <h2>Path JSON Format</h2>
 * <p>Path JSON files contain:
 * <ul>
 *   <li><b>path_elements:</b> Array of translation, rotation, and waypoint targets</li>
 *   <li><b>constraints:</b> Optional path-specific velocity/acceleration constraints</li>
 *   <li><b>default_global_constraints:</b> Optional override for global constraints</li>
 * </ul>
 * 
 * <h2>Usage Examples</h2>
 * <pre>{@code
 * // Load a path from the default autos directory
 * Path path = JsonUtils.loadPath("myPath.json");
 * 
 * // Load a path from a custom directory
 * Path path = JsonUtils.loadPath(new File("/custom/dir"), "myPath.json");
 * 
 * // Load global constraints only
 * Path.DefaultGlobalConstraints globals = JsonUtils.loadGlobalConstraints(JsonUtils.PROJECT_ROOT);
 * }</pre>
 * 
 * @see Path
 * @see Path.DefaultGlobalConstraints
 */
public class JsonUtils {
    /**
     * Container for parsed path components without constructing a full Path object.
     * 
     * <p>This record is useful for separating JSON parsing from Path construction,
     * which can be helpful for performance measurements or when you need to inspect
     * the parsed data before creating a Path.
     * 
     * @param elements The list of parsed path elements
     * @param constraints The parsed path-specific constraints
     * @param defaultGlobalConstraints The default global constraints to use
     */
    public static record ParsedPathComponents(
        ArrayList<PathElement> elements,
        Path.PathConstraints constraints,
        Path.DefaultGlobalConstraints defaultGlobalConstraints
    ) {
        /**
         * Constructs a Path from these parsed components.
         * 
         * <p>This method creates a new Path using the pre-parsed components,
         * avoiding the overhead of JSON parsing.
         * 
         * @return A new Path constructed from the parsed components
         */
        public Path toPath() {
            return new Path(elements, constraints, defaultGlobalConstraints);
        }
    }

    /**
     * The default project root directory for auto routines.
     * 
     * <p>This is set to the "autos" subdirectory within the robot's deploy directory.
     * Path files should be placed in a "paths" subdirectory within this location.
     */
    public static final File PROJECT_ROOT = resolveProjectRoot();

    private static final String[][] PATH_CONSTRAINT_KEY_ALIASES = {
        { "max_velocity_meters_per_sec" },
        { "max_acceleration_meters_per_sec2" },
        { "max_velocity_deg_per_sec" },
        { "max_acceleration_deg_per_sec2" },
        { "end_translation_tolerance_meters" },
        { "end_rotation_tolerance_deg" }
    };

    private static final String[][] GLOBAL_CONSTRAINT_KEY_ALIASES = {
        { "default_max_velocity_meters_per_sec", "max_velocity_meters_per_sec" },
        { "default_max_acceleration_meters_per_sec2", "max_acceleration_meters_per_sec2" },
        { "default_max_velocity_deg_per_sec", "max_velocity_deg_per_sec" },
        { "default_max_acceleration_deg_per_sec2", "max_acceleration_deg_per_sec2" },
        { "default_end_translation_tolerance_meters", "end_translation_tolerance_meters" },
        { "default_end_rotation_tolerance_deg", "end_rotation_tolerance_deg" },
        { "default_intermediate_handoff_radius_meters", "intermediate_handoff_radius_meters" }
    };

    private static final Path.DefaultGlobalConstraints FALLBACK_GLOBAL_CONSTRAINTS =
        new Path.DefaultGlobalConstraints(4.5, 7.0, 720.0, 1500.0, 0.03, 2.0, 0.2);

    private static File resolveProjectRoot() {
        try {
            return new File(Filesystem.getDeployDirectory(), "autos");
        } catch (Throwable ignored) {
            // Allows unit tests or desktop environments that don't include camera server classes.
            return new File("src/main/deploy/autos");
        }
    }

    /**
     * Loads a path from a JSON file in the specified autos directory.
     * 
     * <p>The path file should be located in a "paths" subdirectory within the autos directory.
     * Global constraints are loaded from a "config.json" file in the autos directory.
     * 
     * @param autosDir The directory containing the autos (with paths/ subdirectory)
     * @param pathFileName The name of the path file (including .json extension)
     * @return The loaded Path object
     * @throws RuntimeException if the file cannot be read or parsed
     */
    public static Path loadPath(File autosDir, String pathFileName) {
        try {
            File pathFile = new File(new File(autosDir, "paths"), pathFileName);

            // Read entire file to String (PathPlanner approach)
            String fileContent;
            try (BufferedReader br = new BufferedReader(new FileReader(pathFile))) {
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    sb.append(line);
                }
                fileContent = sb.toString();
            }

            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);
            return buildPathFromJson(json, loadGlobalConstraints(autosDir));
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Failed to load path from " + autosDir.getPath() + "/paths/" + pathFileName, e);
        }
    }

    /**
     * Loads a path from a pre-parsed JSON object with specified global constraints.
     * 
     * @param json The parsed JSON object representing the path
     * @param defaultGlobalConstraints The default global constraints to use
     * @return The loaded Path object
     */
    public static Path loadPath(JSONObject json, Path.DefaultGlobalConstraints defaultGlobalConstraints) {
        return buildPathFromJson(json, defaultGlobalConstraints);
    }

    /**
     * Loads a path from a JSON file in the default project root directory.
     * 
     * <p>This is equivalent to calling {@code loadPath(PROJECT_ROOT, pathFileName)}.
     * 
     * @param pathFileName The name of the path file (including .json extension)
     * @return The loaded Path object
     * @throws RuntimeException if the file cannot be read or parsed
     * @see #PROJECT_ROOT
     */
    public static Path loadPath(String pathFileName) {
        return loadPath(PROJECT_ROOT, pathFileName);
    }

    /**
     * Loads a path from a JSON string with specified global constraints.
     * 
     * <p>This method is useful when the JSON data comes from a source other than a file,
     * such as network communication or embedded resources.
     * 
     * @param pathJson The JSON string representing the path
     * @param defaultGlobalConstraints The default global constraints to use
     * @return The loaded Path object
     * @throws RuntimeException if the JSON string cannot be parsed
     */
    public static Path loadPathFromJsonString(String pathJson, Path.DefaultGlobalConstraints defaultGlobalConstraints) {
        try {
            JSONObject json = (JSONObject) new JSONParser().parse(pathJson);
            return buildPathFromJson(json, defaultGlobalConstraints);
        } catch (ParseException e) {
            throw new RuntimeException("Failed to parse path JSON string", e);
        }
    }

    /**
     * Parses a path JSON object into components without constructing a Path.
     * 
     * <p>This method is useful for performance measurements where you want to separate
     * JSON parsing from Path construction, or when you need to inspect the parsed data
     * before creating a Path.
     * 
     * @param pathJson The JSON object representing the path
     * @param defaultGlobalConstraints Optional default global constraints (can be null,
     *                                  in which case constraints will be loaded from config)
     * @return ParsedPathComponents containing elements, constraints, and globals
     */
    public static ParsedPathComponents parsePathComponents(JSONObject pathJson, Path.DefaultGlobalConstraints defaultGlobalConstraints) {
        ArrayList<PathElement> elements = parsePathElements(pathJson);
        Path.PathConstraints constraints = parsePathConstraints(pathJson);
        
        Path.DefaultGlobalConstraints globals = defaultGlobalConstraints;
        JSONObject globalsJson = (JSONObject) pathJson.get("default_global_constraints");
        if (globalsJson != null) {
            globals = parseDefaultGlobalConstraints(globalsJson);
        } else if (globals == null) {
            globals = loadGlobalConstraints(PROJECT_ROOT);
        }
        
        return new ParsedPathComponents(elements, constraints, globals);
    }

    /**
     * Builds a Path object from a JSON object and global constraints.
     * 
     * @param json The JSON object containing path data
     * @param defaultGlobalConstraints The default global constraints to use
     * @return The constructed Path object
     */
    private static Path buildPathFromJson(JSONObject json, Path.DefaultGlobalConstraints defaultGlobalConstraints) {
        ArrayList<PathElement> elements = parsePathElements(json);

        Path.PathConstraints constraints = parsePathConstraints(json);

        Path.DefaultGlobalConstraints globals = defaultGlobalConstraints;

        JSONObject globalsJson = (JSONObject) json.get("default_global_constraints");
        if (globalsJson != null) {
            globals = parseDefaultGlobalConstraints(globalsJson);
        } else if (globals == null) {
            globals = loadGlobalConstraints(PROJECT_ROOT);
        }

        return new Path(elements, constraints, globals);
    }

    /**
     * Parses path elements from a JSON object.
     * 
     * <p>Supports three element types:
     * <ul>
     *   <li><b>translation:</b> A position target with optional handoff radius</li>
     *   <li><b>rotation:</b> A holonomic rotation target with t_ratio and optional profiling</li>
     *   <li><b>waypoint:</b> Combined translation and rotation target</li>
     * </ul>
     * 
     * @param json The JSON object containing path_elements array
     * @return ArrayList of parsed PathElement objects
     */
    private static ArrayList<PathElement> parsePathElements(JSONObject json) {
        ArrayList<PathElement> elements = new ArrayList<>();
        JSONArray pathElementsJson = (JSONArray) json.get("path_elements");
        if (pathElementsJson == null) {
            return elements;
        }

        for (Object obj : pathElementsJson) {
            if (!(obj instanceof JSONObject)) {
                continue;
            }
            JSONObject elementJson = (JSONObject) obj;
            String type = (String) elementJson.get("type");

            if ("translation".equals(type)) {
                double xMeters = ((Number) elementJson.get("x_meters")).doubleValue();
                double yMeters = ((Number) elementJson.get("y_meters")).doubleValue();
                Object handoffObj = elementJson.get("intermediate_handoff_radius_meters");
                Double handoff = handoffObj != null ? ((Number) handoffObj).doubleValue() : null;

                elements.add(new TranslationTarget(
                    new Translation2d(xMeters, yMeters),
                    Optional.ofNullable(handoff)
                ));
            } else if ("rotation".equals(type)) {
                double rotationRadians = ((Number) elementJson.get("rotation_radians")).doubleValue();
                Object tRatioObj = elementJson.get("t_ratio");
                double tRatio = tRatioObj != null ? ((Number) tRatioObj).doubleValue() : 0.5;
                Object profiledObj = elementJson.get("profiled_rotation");
                boolean profiled = profiledObj != null && (Boolean) profiledObj;

                elements.add(new RotationTarget(
                    Rotation2d.fromRadians(rotationRadians),
                    tRatio,
                    profiled
                ));
            } else if ("event_trigger".equals(type)) {
                Object tRatioObj = elementJson.get("t_ratio");
                double tRatio = tRatioObj != null ? ((Number) tRatioObj).doubleValue() : 0.5;
                String libKey = (String) elementJson.get("lib_key");
                if (libKey == null) {
                    continue;
                }
                elements.add(new EventTrigger(tRatio, libKey));
            } else if ("waypoint".equals(type)) {
                JSONObject translationJson = (JSONObject) elementJson.get("translation_target");
                if (translationJson == null) {
                    continue;
                }
                double txMeters = ((Number) translationJson.get("x_meters")).doubleValue();
                double tyMeters = ((Number) translationJson.get("y_meters")).doubleValue();
                Object tHandoffObj = translationJson.get("intermediate_handoff_radius_meters");
                Double tHandoff = tHandoffObj != null ? ((Number) tHandoffObj).doubleValue() : null;

                JSONObject rotationJson = (JSONObject) elementJson.get("rotation_target");
                if (rotationJson == null) {
                    continue;
                }
                double rotRadians = ((Number) rotationJson.get("rotation_radians")).doubleValue();
                Object rTRatioObj = rotationJson.get("t_ratio");
                double rTRatio = rTRatioObj != null ? ((Number) rTRatioObj).doubleValue() : 0.5;
                Object rProfiledObj = rotationJson.get("profiled_rotation");
                boolean rProfiled = rProfiledObj != null && (Boolean) rProfiledObj;

                TranslationTarget t = new TranslationTarget(
                    new Translation2d(txMeters, tyMeters),
                    Optional.ofNullable(tHandoff)
                );
                RotationTarget r = new RotationTarget(
                    Rotation2d.fromRadians(rotRadians),
                    rTRatio,
                    rProfiled
                );
                elements.add(new Waypoint(t, r));
            }
        }
        return elements;
    }

    /**
     * Parses path constraints from a JSON object.
     * 
     * <p>Constraints are optional and can include:
     * <ul>
     *   <li>max_velocity_meters_per_sec</li>
     *   <li>max_acceleration_meters_per_sec2</li>
     *   <li>max_velocity_deg_per_sec</li>
     *   <li>max_acceleration_deg_per_sec2</li>
     *   <li>end_translation_tolerance_meters</li>
     *   <li>end_rotation_tolerance_deg</li>
     * </ul>
     * 
     * @param json The JSON object containing constraints
     * @return PathConstraints object with parsed values
     */
    private static Path.PathConstraints parsePathConstraints(JSONObject json) {
        Path.PathConstraints constraints = new Path.PathConstraints();
        JSONObject constraintsJson = getNestedObject(json, "constraints");
        if (constraintsJson == null) {
            constraintsJson = findBestObjectContainingKeys(json, PATH_CONSTRAINT_KEY_ALIASES);
        }

        parseConstraint(constraintsJson, json, "max_velocity_meters_per_sec", (val) -> {
            if (val.isPresent()) {
                constraints.setMaxVelocityMetersPerSec(val.get().toArray(new Path.RangedConstraint[0]));
            }
        });
        parseConstraint(constraintsJson, json, "max_acceleration_meters_per_sec2", (val) -> {
            if (val.isPresent()) {
                constraints.setMaxAccelerationMetersPerSec2(val.get().toArray(new Path.RangedConstraint[0]));
            }
        });
        parseConstraint(constraintsJson, json, "max_velocity_deg_per_sec", (val) -> {
            if (val.isPresent()) {
                constraints.setMaxVelocityDegPerSec(val.get().toArray(new Path.RangedConstraint[0]));
            }
        });
        parseConstraint(constraintsJson, json, "max_acceleration_deg_per_sec2", (val) -> {
            if (val.isPresent()) {
                constraints.setMaxAccelerationDegPerSec2(val.get().toArray(new Path.RangedConstraint[0]));
            }
        });

        lookupValueByKeys(constraintsJson, json, "end_translation_tolerance_meters")
            .flatMap(JsonUtils::toDouble)
            .ifPresent(constraints::setEndTranslationToleranceMeters);
        lookupValueByKeys(constraintsJson, json, "end_rotation_tolerance_deg")
            .flatMap(JsonUtils::toDouble)
            .ifPresent(constraints::setEndRotationToleranceDeg);

        return constraints;
    }

    /**
     * Parses default global constraints from a JSON object.
     * 
     * @param json The JSON object containing global constraint values
     * @return DefaultGlobalConstraints with all required values
     */
    private static Path.DefaultGlobalConstraints parseDefaultGlobalConstraints(JSONObject json) {
        JSONObject constraintsJson = getNestedObject(json, "kinematic_constraints");
        if (constraintsJson == null) {
            constraintsJson = findBestObjectContainingKeys(json, GLOBAL_CONSTRAINT_KEY_ALIASES);
        }

        double dMaxVelMps = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_max_velocity_meters_per_sec",
            FALLBACK_GLOBAL_CONSTRAINTS.getMaxVelocityMetersPerSec(),
            "max_velocity_meters_per_sec"
        );
        double dMaxAccMps2 = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_max_acceleration_meters_per_sec2",
            FALLBACK_GLOBAL_CONSTRAINTS.getMaxAccelerationMetersPerSec2(),
            "max_acceleration_meters_per_sec2"
        );
        double dMaxVelDeg = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_max_velocity_deg_per_sec",
            FALLBACK_GLOBAL_CONSTRAINTS.getMaxVelocityDegPerSec(),
            "max_velocity_deg_per_sec"
        );
        double dMaxAccDeg2 = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_max_acceleration_deg_per_sec2",
            FALLBACK_GLOBAL_CONSTRAINTS.getMaxAccelerationDegPerSec2(),
            "max_acceleration_deg_per_sec2"
        );
        double endTransTol = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_end_translation_tolerance_meters",
            FALLBACK_GLOBAL_CONSTRAINTS.getEndTranslationToleranceMeters(),
            "end_translation_tolerance_meters"
        );
        double endRotTolDeg = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_end_rotation_tolerance_deg",
            FALLBACK_GLOBAL_CONSTRAINTS.getEndRotationToleranceDeg(),
            "end_rotation_tolerance_deg"
        );
        double handoffRadius = readDoubleConstraintValue(
            constraintsJson,
            json,
            "default_intermediate_handoff_radius_meters",
            FALLBACK_GLOBAL_CONSTRAINTS.getIntermediateHandoffRadiusMeters(),
            "intermediate_handoff_radius_meters"
        );

        return new Path.DefaultGlobalConstraints(
            dMaxVelMps,
            dMaxAccMps2,
            dMaxVelDeg,
            dMaxAccDeg2,
            endTransTol,
            endRotTolDeg,
            handoffRadius
        );
    }

    /**
     * Parses a ranged constraint array from JSON.
     * 
     * @param constraintsJson The constraints JSON object
     * @param key The key for the constraint array
     * @param setter Consumer to set the parsed constraint values
     */
    private static void parseConstraint(
        JSONObject constraintsJson,
        JSONObject rootJson,
        String key,
        Consumer<Optional<ArrayList<Path.RangedConstraint>>> setter
    ) {
        Optional<Object> arrObj = lookupValueByKeys(constraintsJson, rootJson, key);
        if (arrObj.isEmpty() || !(arrObj.get() instanceof JSONArray arr) || arr.isEmpty()) {
            return;
        }

        ArrayList<Path.RangedConstraint> list = new ArrayList<>();
        for (Object obj : arr) {
            if (!(obj instanceof JSONObject rcJson)) {
                continue;
            }
            Optional<Double> value = toDouble(rcJson.get("value"));
            Optional<Integer> startOrdinal = toInt(rcJson.get("start_ordinal"));
            Optional<Integer> endOrdinal = toInt(rcJson.get("end_ordinal"));
            if (value.isEmpty() || startOrdinal.isEmpty() || endOrdinal.isEmpty()) {
                continue;
            }
            list.add(new Path.RangedConstraint(value.get(), startOrdinal.get(), endOrdinal.get()));
        }
        if (!list.isEmpty()) {
            setter.accept(Optional.of(list));
        }
    }

    /**
     * Loads global constraints from a config.json file in the specified directory.
     * 
     * <p>The config.json file should contain default values for all constraint types:
     * <ul>
     *   <li>default_max_velocity_meters_per_sec</li>
     *   <li>default_max_acceleration_meters_per_sec2</li>
     *   <li>default_max_velocity_deg_per_sec</li>
     *   <li>default_max_acceleration_deg_per_sec2</li>
     *   <li>default_end_translation_tolerance_meters</li>
     *   <li>default_end_rotation_tolerance_deg</li>
     *   <li>default_intermediate_handoff_radius_meters</li>
     * </ul>
     * 
     * @param autosDir The directory containing config.json
     * @return DefaultGlobalConstraints loaded from the config file
     * @throws RuntimeException if the config file cannot be read or parsed
     */
    public static Path.DefaultGlobalConstraints loadGlobalConstraints(File autosDir) {
        try {
            File config = new File(autosDir, "config.json");

            // Read entire file to String (PathPlanner approach)
            String fileContent;
            try (BufferedReader br = new BufferedReader(new FileReader(config))) {
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = br.readLine()) != null) {
                    sb.append(line);
                }
                fileContent = sb.toString();
            }

            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);
            return parseDefaultGlobalConstraints(json);
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Failed to load global constraints from " + autosDir.getPath() + "/config.json", e);
        }
    }

    private static JSONObject getNestedObject(JSONObject json, String key) {
        Object val = json.get(key);
        return val instanceof JSONObject ? (JSONObject) val : null;
    }

    private static Optional<Object> lookupValueByKeys(
        JSONObject preferredContainer,
        JSONObject rootJson,
        String... keys
    ) {
        for (String key : keys) {
            Optional<Object> preferred = lookupDirect(preferredContainer, key);
            if (preferred.isPresent()) {
                return preferred;
            }
        }
        for (String key : keys) {
            Optional<Object> direct = lookupDirect(rootJson, key);
            if (direct.isPresent()) {
                return direct;
            }
        }
        for (String key : keys) {
            Optional<Object> recursive = findFirstValueByKey(rootJson, key);
            if (recursive.isPresent()) {
                return recursive;
            }
        }
        return Optional.empty();
    }

    private static Optional<Object> lookupDirect(JSONObject json, String key) {
        if (json == null || !json.containsKey(key)) {
            return Optional.empty();
        }
        Object value = json.get(key);
        return value == null ? Optional.empty() : Optional.of(value);
    }

    private static Optional<Object> findFirstValueByKey(Object node, String key) {
        if (node instanceof JSONObject obj) {
            if (obj.containsKey(key)) {
                Object value = obj.get(key);
                if (value != null) {
                    return Optional.of(value);
                }
            }
            for (Object child : obj.values()) {
                Optional<Object> found = findFirstValueByKey(child, key);
                if (found.isPresent()) {
                    return found;
                }
            }
        } else if (node instanceof JSONArray arr) {
            for (Object child : arr) {
                Optional<Object> found = findFirstValueByKey(child, key);
                if (found.isPresent()) {
                    return found;
                }
            }
        }
        return Optional.empty();
    }

    private static JSONObject findBestObjectContainingKeys(JSONObject rootJson, String[][] keyAliases) {
        ArrayList<JSONObject> objects = new ArrayList<>();
        collectJsonObjects(rootJson, objects);

        JSONObject best = null;
        int bestScore = 0;
        for (JSONObject candidate : objects) {
            int score = countMatchingKeys(candidate, keyAliases);
            if (score > bestScore) {
                bestScore = score;
                best = candidate;
            }
        }
        return best;
    }

    private static void collectJsonObjects(Object node, ArrayList<JSONObject> out) {
        if (node instanceof JSONObject obj) {
            out.add(obj);
            for (Object child : obj.values()) {
                collectJsonObjects(child, out);
            }
        } else if (node instanceof JSONArray arr) {
            for (Object child : arr) {
                collectJsonObjects(child, out);
            }
        }
    }

    private static int countMatchingKeys(JSONObject json, String[][] keyAliases) {
        int score = 0;
        for (String[] aliases : keyAliases) {
            for (String alias : aliases) {
                if (json.containsKey(alias) && json.get(alias) != null) {
                    score++;
                    break;
                }
            }
        }
        return score;
    }

    private static Optional<Double> toDouble(Object value) {
        if (value instanceof Number number) {
            return Optional.of(number.doubleValue());
        }
        if (value instanceof String text) {
            try {
                return Optional.of(Double.parseDouble(text.trim()));
            } catch (NumberFormatException ignored) {
                return Optional.empty();
            }
        }
        return Optional.empty();
    }

    private static Optional<Integer> toInt(Object value) {
        if (value instanceof Number number) {
            return Optional.of(number.intValue());
        }
        if (value instanceof String text) {
            try {
                return Optional.of(Integer.parseInt(text.trim()));
            } catch (NumberFormatException ignored) {
                return Optional.empty();
            }
        }
        return Optional.empty();
    }

    private static double readDoubleConstraintValue(
        JSONObject preferredContainer,
        JSONObject rootJson,
        String primaryKey,
        double fallbackValue,
        String... aliases
    ) {
        String[] keys = new String[aliases.length + 1];
        keys[0] = primaryKey;
        System.arraycopy(aliases, 0, keys, 1, aliases.length);

        Optional<Object> raw = lookupValueByKeys(preferredContainer, rootJson, keys);
        if (raw.isEmpty()) {
            System.err.println(
                "BLine JsonUtils: Missing constraint key '" + primaryKey + "', using fallback value " + fallbackValue
            );
            return fallbackValue;
        }
        Optional<Double> parsed = toDouble(raw.get());
        if (parsed.isEmpty()) {
            System.err.println(
                "BLine JsonUtils: Constraint key '" + primaryKey + "' must be numeric, using fallback value " + fallbackValue
            );
            return fallbackValue;
        }
        return parsed.get();
    }

}
