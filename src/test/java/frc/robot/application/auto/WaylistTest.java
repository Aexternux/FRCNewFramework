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
}
