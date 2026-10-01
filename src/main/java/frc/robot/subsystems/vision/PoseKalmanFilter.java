package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Vision-only filter used to assess the consistency of consecutive camera measurements.
 *
 * <p>This filter intentionally does not consume drivetrain odometry. The drivetrain's
 * {@code SwerveDrivePoseEstimator} is the sole pose estimator that combines wheel/gyro odometry
 * with vision. Keeping this filter vision-only avoids feeding a second, correlated pose estimate
 * back into that estimator.
 */
public class PoseKalmanFilter {

    /** Result of testing one camera measurement against recent vision history. */
    public record GateResult(boolean accepted, double stdDevMultiplier, boolean updated,
            double innovationSigma) {}

    private double x;
    private double y;
    private double theta;

    // Diagonal covariance for this vision-only history.
    private double px;
    private double py;
    private double pTheta;

    // Process variance added per second. Units are m^2/s and rad^2/s.
    private double qxPerSecond;
    private double qyPerSecond;
    private double qThetaPerSecond;

    private double lastTimestamp_s = Double.NaN;
    private boolean initialized = false;

    public PoseKalmanFilter(double processStdTranslation_mPerRootS,
            double processStdRotation_radPerRootS) {
        setProcessNoise(processStdTranslation_mPerRootS, processStdRotation_radPerRootS);
    }

    /**
     * Sets process standard deviations in units per square-root second. Larger values allow the
     * vision history to move farther between camera frames before it rejects an observation.
     */
    public void setProcessNoise(double stdTranslation_mPerRootS,
            double stdRotation_radPerRootS) {
        qxPerSecond = stdTranslation_mPerRootS * stdTranslation_mPerRootS;
        qyPerSecond = qxPerSecond;
        qThetaPerSecond = stdRotation_radPerRootS * stdRotation_radPerRootS;
    }

    /** Clears history so the next accepted vision observation establishes a new reference. */
    public void clear() {
        initialized = false;
        lastTimestamp_s = Double.NaN;
    }

    /**
     * Checks and records a measurement at its image-capture timestamp.
     *
     * <p>Repeated or out-of-order measurements are dropped. This prevents a NetworkTables value
     * that is unchanged across several robot loops from being fused into WPILib repeatedly.
     */
    public GateResult evaluateAndUpdate(Pose2d measurement, double timestamp_s,
            double stdX_m, double stdY_m, double stdTheta_rad, double maxInnovationSigma,
            double maxStdDevMultiplier) {
        if (!initialized) {
            reset(measurement, timestamp_s, stdX_m, stdY_m, stdTheta_rad);
            return new GateResult(true, 1.0, true, 0.0);
        }

        if (timestamp_s <= lastTimestamp_s) {
            return new GateResult(false, 1.0, false, Double.NaN);
        }

        predictTo(timestamp_s);

        double innovationX = measurement.getX() - x;
        double innovationY = measurement.getY() - y;
        double innovationTheta = MathUtil.angleModulus(measurement.getRotation().getRadians() - theta);

        double sigmaX = Math.sqrt(px + stdX_m * stdX_m);
        double sigmaY = Math.sqrt(py + stdY_m * stdY_m);
        double sigmaTheta = Math.sqrt(pTheta + stdTheta_rad * stdTheta_rad);
        double normalizedInnovation = Math.max(
                Math.abs(innovationX) / sigmaX,
                Math.max(Math.abs(innovationY) / sigmaY, Math.abs(innovationTheta) / sigmaTheta));

        if (normalizedInnovation > maxInnovationSigma) {
            return new GateResult(false, 1.0, false, normalizedInnovation);
        }

        // Near the gate threshold, keep the observation but make its handoff to WPILib less
        // influential. The original camera pose/timestamp are still preserved.
        double multiplier = Math.min(maxStdDevMultiplier,
                1.0 + normalizedInnovation / maxInnovationSigma * (maxStdDevMultiplier - 1.0));
        double adjustedStdX = stdX_m * multiplier;
        double adjustedStdY = stdY_m * multiplier;
        double adjustedStdTheta = stdTheta_rad * multiplier;
        update(measurement, adjustedStdX * adjustedStdX, adjustedStdY * adjustedStdY,
                adjustedStdTheta * adjustedStdTheta);
        return new GateResult(true, multiplier, true, normalizedInnovation);
    }

    private void reset(Pose2d pose, double timestamp_s, double stdX_m, double stdY_m,
            double stdTheta_rad) {
        x = pose.getX();
        y = pose.getY();
        theta = pose.getRotation().getRadians();
        px = stdX_m * stdX_m;
        py = stdY_m * stdY_m;
        pTheta = stdTheta_rad * stdTheta_rad;
        lastTimestamp_s = timestamp_s;
        initialized = true;
    }

    /** No motion model: this gate relies only on independent camera history. */
    private void predictTo(double timestamp_s) {
        double dt_s = timestamp_s - lastTimestamp_s;
        px += qxPerSecond * dt_s;
        py += qyPerSecond * dt_s;
        pTheta += qThetaPerSecond * dt_s;
        lastTimestamp_s = timestamp_s;
    }

    private void update(Pose2d measurement, double rx, double ry, double rTheta) {
        double kx = px / (px + rx);
        double ky = py / (py + ry);
        double kTheta = pTheta / (pTheta + rTheta);

        x += kx * (measurement.getX() - x);
        y += ky * (measurement.getY() - y);
        theta += kTheta * MathUtil.angleModulus(measurement.getRotation().getRadians() - theta);

        px *= (1.0 - kx);
        py *= (1.0 - ky);
        pTheta *= (1.0 - kTheta);
    }

    public boolean isInitialized() {
        return initialized;
    }

    public Pose2d getEstimate() {
        return new Pose2d(x, y, new Rotation2d(theta));
    }
}
