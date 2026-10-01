package frc.robot.subsystems.vision;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Layer 1 of pose estimation: fuse any number of independent limelight pose measurements into a
 * single pose using inverse-variance weighting (each axis weighted by 1/variance). The fused
 * variance is the harmonic combination 1 / sum(1/variance_i).
 */
public final class VisionFusion {

    private VisionFusion() {}

    /** One limelight's pose measurement with diagonal variances and a capture timestamp. */
    public static record Measurement(
            Pose2d pose, double varX, double varY, double varTheta, double timestamp) {}

    /**
     * Inverse-variance fuse the measurements. Returns {@code null} for an empty list. Theta is
     * fused relative to the first measurement's angle to stay wrap-safe; if every measurement
     * declines to weigh in on rotation (infinite/huge variance), the fused rotation falls back to
     * that reference with effectively infinite variance.
     */
    public static Measurement fuse(List<Measurement> measurements) {
        if (measurements.isEmpty()) {
            return null;
        }
        if (measurements.size() == 1) {
            return measurements.get(0);
        }

        double invSumX = 0.0;
        double invSumY = 0.0;
        double invSumTheta = 0.0;
        double weightedX = 0.0;
        double weightedY = 0.0;
        double weightedTheta = 0.0; // accumulated relative to refTheta
        double timestampSum = 0.0;

        double refTheta = measurements.get(0).pose().getRotation().getRadians();

        for (Measurement m : measurements) {
            double ix = 1.0 / m.varX();
            double iy = 1.0 / m.varY();
            double it = 1.0 / m.varTheta();

            invSumX += ix;
            invSumY += iy;
            invSumTheta += it;

            weightedX += m.pose().getX() * ix;
            weightedY += m.pose().getY() * iy;
            weightedTheta +=
                    MathUtil.angleModulus(m.pose().getRotation().getRadians() - refTheta) * it;

            timestampSum += m.timestamp();
        }

        double fusedX = weightedX / invSumX;
        double fusedY = weightedY / invSumY;

        double fusedTheta;
        double varTheta;
        if (invSumTheta > 0.0) {
            fusedTheta = refTheta + weightedTheta / invSumTheta;
            varTheta = 1.0 / invSumTheta;
        } else {
            fusedTheta = refTheta;
            varTheta = Double.POSITIVE_INFINITY;
        }

        Pose2d fusedPose = new Pose2d(fusedX, fusedY, new Rotation2d(fusedTheta));
        return new Measurement(
                fusedPose,
                1.0 / invSumX,
                1.0 / invSumY,
                varTheta,
                timestampSum / measurements.size());
    }
}
