package frc.robot.application.auto;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

class WaylistTest {
    @Test
    void completionRequiresFinalHeadingAsWellAsTranslation() {
        Waylist path = new Waylist(new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90.0)), 0.1);

        assertFalse(path.isComplete(new Pose2d(2.02, 2.98, new Rotation2d()), 0.1,
                Math.toRadians(5.0)));
        assertTrue(path.isComplete(new Pose2d(2.02, 2.98, Rotation2d.fromDegrees(88.0)), 0.1,
                Math.toRadians(5.0)));
    }

    @Test
    void completionRejectsTranslationOutsideTolerance() {
        Waylist path = new Waylist(new Pose2d(2.0, 3.0, new Rotation2d()), 0.1);

        assertFalse(path.isComplete(new Pose2d(2.11, 3.0, new Rotation2d()), 0.1,
                Math.toRadians(5.0)));
    }

    @Test
    void completionUsesTheWaypointTolerance() {
        Waylist path = new Waylist(new Pose2d(2.0, 3.0, new Rotation2d()), 0.25);

        assertTrue(path.isCompleteWithHeading(new Pose2d(2.20, 3.0, new Rotation2d()),
                Math.toRadians(5.0)));
    }

    @Test
    void targetIsLookaheadAheadWhenOnThePath() {
        Waylist path = new Waylist(new Pose2d(0.0, 0.0, new Rotation2d()), 0.1)
                .append(new Pose2d(5.0, 0.0, new Rotation2d()), 0.1);

        Pose2d target = path.getBlendedTarget(new Pose2d(1.0, 0.0, new Rotation2d()), 1.2);

        assertTrue(Math.abs(target.getX() - 2.2) < 1e-9,
                "expected 1.2m ahead of the on-path robot, got " + target.getX());
    }

    @Test
    void targetDoesNotSkipTheFirstPoseWhenStartingOffPath() {
        Waylist path = new Waylist(new Pose2d(5.995, 7.478, Rotation2d.fromDegrees(-90.0)), 0.1)
                .append(new Pose2d(5.911, 4.780, Rotation2d.fromDegrees(-88.0)), 0.1);

        Pose2d target = path.getBlendedTarget(new Pose2d(2.950, 7.478, new Rotation2d()), 1.2);

        assertTrue(target.getTranslation().getDistance(
                new Pose2d(5.995, 7.478, new Rotation2d()).getTranslation()) < 0.2,
                "expected the target to stay near the waylist start, got " + target);
    }
}
