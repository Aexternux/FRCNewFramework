package frc.robot.subsystems.tracking;

import java.util.HashMap;
import java.util.Map;
import java.util.OptionalDouble;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/**
 * Maps each AprilTag id to the field-relative robot heading (degrees) the drive should align to
 * when the TRACKING pathing override is active and that tag is the primary target.
 *
 * <p>Defaults are derived from the field layout: for each tag the target heading is "face the tag
 * square-on" — the tag's outward normal reversed, i.e. the heading a robot pointed straight at the
 * tag would hold. Put entries in {@link #OVERRIDES} to force a specific heading for individual tags
 * (e.g. a scoring face you want to approach at an angle); those take precedence over the default.
 *
 * <p>A tag id with no default and no override is treated as "not tracked": {@link #getTargetAngle}
 * returns empty, and the tracking override leaves the driver's input untouched for that tag.
 */
public final class TagAngles {

    private TagAngles() {}

    private static final AprilTagFieldLayout LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    /**
     * Manual per-tag heading overrides (tag id -> field-relative degrees). Empty by default; add an
     * entry only when the "face the tag" default isn't the heading you want for that tag.
     */
    private static final Map<Integer, Double> OVERRIDES = Map.ofEntries(
            // Example: Map.entry(18, -60.0)
    );

    // tag id -> target robot heading in degrees, field-relative.
    private static final Map<Integer, Double> TARGET_ANGLES_deg = buildTargetAngles();

    private static Map<Integer, Double> buildTargetAngles() {
        Map<Integer, Double> map = new HashMap<>();
        map.put(1, 0.0);
        map.put(2, 90.0);
        map.put(3, 0.0);
        map.put(4, 0.0);
        map.put(5, 270.0);
        map.put(6, 0.0);
        map.put(7, 180.0);
        map.put(8, 270.0);
        map.put(9, 180.0);
        map.put(10, 180.0);
        map.put(11, 270.0);
        map.put(12, 180.0);
        map.put(13, 0.0);
        map.put(14, 0.0);
        map.put(15, 0.0);
        map.put(16, 0.0);
        map.put(17, 180.0);
        map.put(18, 90.0);
        map.put(19, 180.0);
        map.put(20, 180.0);
        map.put(21, 270.0);
        map.put(22, 180.0);
        map.put(23, 0.0);
        map.put(24, 270.0);
        map.put(25, 0.0);
        map.put(26, 0.0);
        map.put(27, 90.0);
        map.put(28, 0.0);
        map.put(29, 180.0);
        map.put(30, 180.0);
        map.put(31, 180.0);
        map.put(32, 180.0);
        map.putAll(OVERRIDES);
        return Map.copyOf(map);
    }

    /**
     * Target robot heading (degrees) to face when tracking the given tag, or empty if the tag has
     * no configured angle.
     */
    public static OptionalDouble getTargetAngle(int tagId) {
        Double angle = TARGET_ANGLES_deg.get(tagId);
        return angle == null ? OptionalDouble.empty() : OptionalDouble.of(angle);
    }

    /** Whether the given tag id has a configured tracking angle. */
    public static boolean isTracked(int tagId) {
        return TARGET_ANGLES_deg.containsKey(tagId);
    }
}
