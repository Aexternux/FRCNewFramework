package frc.robot.devices.limelight;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Hardware abstraction for a Limelight smart camera (tracking + AprilTag localization). */
public interface LimelightIO {

    @AutoLog
    public static class LimelightIOInputs {
        public boolean connected = false;
        public boolean hasTarget = false;

        // ---- Tracking (best target relative to the camera crosshair) ----
        public double tx = 0.0; // horizontal offset, degrees (negative = left)
        public double ty = 0.0; // vertical offset, degrees (negative = down)
        public double ta = 0.0; // target area, percent of image (0-100)
        public int primaryTagId = -1;
        public int[] visibleTagIds = new int[0];

        // Best target pose in camera space (meters): x = lateral offset, z = forward distance.
        public double targetSpaceX_m = 0.0;
        public double targetSpaceZ_m = 0.0;

        // ---- Pose estimation (field-relative AprilTag estimates) ----
        public PoseObservation[] poseObservations = new PoseObservation[0];

        public static record PoseObservation(
                double timestamp,
                Pose3d pose,
                double ambiguity,
                int tagCount,
                double avgTagDistance,
                PoseType type) {}

        public enum PoseType {
            MEGATAG_1,
            MEGATAG_2
        }
    }

    public default void updateInputs(LimelightIOInputs inputs) {}

    public default void setPipeline(int index) {}

    public default void setFiducialFilters(int[] ids) {}

    /** Feed the current robot heading (from odometry) for MegaTag2 fusion. */
    public default void setRobotOrientation(Rotation2d heading) {}

    /** Simulation only: provide the ground-truth robot pose so the camera can render tags. */
    public default void updateSimPose(Pose2d robotPose) {}
}
