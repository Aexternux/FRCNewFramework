package frc.robot.application.auto;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.application.superstructure.SS.Flag;
import frc.robot.constants.Enums.StartLocation;

/** Declarative sequence of drive and timed superstructure actions. */
public class ActionList {
    private static final double DEFAULT_DRIVE_TIMEOUT_s = 15.0;
    private static final double START_ACTION_TOLERANCE_m = 0.1;
    private static final double START_ACTION_DELAY_s = 0.34;
    public enum ActionType { DRIVE, WAIT }
    public enum PathingStyle { PRECISE, CONTINUOUS }

    public static class Action {
        public final ActionType type;
        public final Waylist waylist;
        public final double maxVelocity_mps;
        public final double translation_kP;
        public final double rotation_kP;
        public final double timeout_s;
        public final double minimumDwell_s;
        public final EnumSet<Flag> flags;
        public final PathingStyle pathingStyle;

        private Action(ActionType type, Waylist waylist, double maxVelocity_mps, double translation_kP,
                double rotation_kP, double timeout_s, double minimumDwell_s, EnumSet<Flag> flags,
                PathingStyle pathingStyle) {
            this.type = type;
            this.waylist = waylist;
            this.maxVelocity_mps = maxVelocity_mps;
            this.translation_kP = translation_kP;
            this.rotation_kP = rotation_kP;
            this.timeout_s = timeout_s;
            this.minimumDwell_s = minimumDwell_s;
            this.flags = EnumSet.copyOf(flags);
            this.pathingStyle = pathingStyle;
        }
    }

    private final String name;
    private final StartLocation startLocation;
    private final Pose2d explicitStartPose;
    private final PathingStyle pathingStyle;
    private final ArrayList<Action> actions = new ArrayList<>();
    private int index = 0;

    public ActionList(String name, StartLocation startLocation) {
        this(name, startLocation, PathingStyle.PRECISE);
    }

    /** Selects whether this routine settles at every waypoint or carries speed through them. */
    public ActionList(String name, StartLocation startLocation, PathingStyle pathingStyle) {
        this.name = name;
        this.startLocation = startLocation;
        this.explicitStartPose = null;
        this.pathingStyle = pathingStyle;
    }

    /** Creates an auto that resets odometry to this exact blue-alliance field pose. */
    public ActionList(String name, Pose2d startPose, PathingStyle pathingStyle) {
        this.name = name;
        this.startLocation = null;
        this.explicitStartPose = startPose;
        this.pathingStyle = pathingStyle;
    }

    public ActionList driveToPose(Waylist waylist, double maxVelocity_mps, double translation_kP,
            double rotation_kP) {
        return driveToPose(waylist, maxVelocity_mps, translation_kP, rotation_kP,
                DEFAULT_DRIVE_TIMEOUT_s, 0.0);
    }

    public ActionList driveToPose(Waylist waylist, double maxVelocity_mps, double translation_kP,
            double rotation_kP, double timeout_s, double minimumDwell_s, Flag... flags) {
        actions.add(new Action(ActionType.DRIVE, waylist, maxVelocity_mps, translation_kP, rotation_kP, timeout_s,
                minimumDwell_s, enumSet(flags), pathingStyle));
        return this;
    }

    public ActionList waitFor(double timeout_s, Flag... flags) {
        actions.add(new Action(ActionType.WAIT, null, 0.0, 0.0, 0.0, timeout_s, 0.0,
                enumSet(flags), pathingStyle));
        return this;
    }

    public ActionList startActions() {
        actions.add(new Action(ActionType.WAIT, new Waylist(startPose(), START_ACTION_TOLERANCE_m),
                0.0, 0.0, 0.0, START_ACTION_DELAY_s, 0.0, EnumSet.noneOf(Flag.class), pathingStyle));
        return this;
    }

    private static EnumSet<Flag> enumSet(Flag... flags) {
        EnumSet<Flag> result = EnumSet.noneOf(Flag.class);
        for (Flag flag : flags) result.add(flag);
        return result;
    }

    public void reset() {
        index = 0;
        for (Action action : actions) if (action.waylist != null) action.waylist.reset();
    }

    public void transformWaylists(Function<Pose2d, Pose2d> transform) {
        for (Action action : actions) {
            if (action.waylist != null) action.waylist.transformAll(transform);
        }
    }

    public Action currentAction() { return index < actions.size() ? actions.get(index) : null; }
    public void pass() { index++; }
    public int getIndex() { return index; }
    public int size() { return actions.size(); }
    public String getName() { return name; }
    public StartLocation getStartLocation() { return startLocation; }
    public String getStartDescription() {
        return explicitStartPose == null ? startLocation.name() : "CUSTOM";
    }
    public Pose2d startPose() {
        if (explicitStartPose != null) return explicitStartPose;
        return switch (startLocation) {
            case LEFT -> PoseBank.LEFT;
            case CENTER -> PoseBank.CENTER;
            case RIGHT -> PoseBank.RIGHT;
        };
    }
}
