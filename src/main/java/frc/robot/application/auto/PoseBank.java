package frc.robot.application.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Starting poses and named field poses for hand-authored autonomous routines. */
public final class PoseBank {
    private PoseBank() {}

    /** Blue-alliance starting poses, matching the three reference auto stations. */
    public static final Pose2d LEFT = new Pose2d(4.450, 7.612, Rotation2d.fromDegrees(180));
    public static final Pose2d CENTER = new Pose2d(3.600, 4.197, Rotation2d.fromDegrees(270));
    public static final Pose2d RIGHT = new Pose2d(4.490, 0.458, Rotation2d.fromDegrees(180));
}
