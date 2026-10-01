package frc.robot.subsystems.vision;

/**
 * Tracks short-term vision consistency and converts it into a conservative measurement-noise
 * multiplier.
 *
 * <p>This class has no drivetrain input and never creates a replacement robot pose. Its only
 * output is a multiplier for the standard deviations sent with an already accepted camera pose.
 */
public class VisionTrustTracker {

    /** Result used for logging and the covariance handoff to the drivetrain estimator. */
    public record TrustResult(double stdDevMultiplier, double jitterSigma, boolean warmedUp) {}

    private final double timeConstant_s;
    private final int warmupSamples;
    private final double minMultiplier;
    private final double maxMultiplier;

    private double lastTimestamp_s = Double.NaN;
    private double meanSquaredInnovation = 1.0;
    private int sampleCount = 0;

    public VisionTrustTracker(double timeConstant_s, int warmupSamples, double minMultiplier,
            double maxMultiplier) {
        this.timeConstant_s = timeConstant_s;
        this.warmupSamples = warmupSamples;
        this.minMultiplier = minMultiplier;
        this.maxMultiplier = maxMultiplier;
    }

    public void clear() {
        lastTimestamp_s = Double.NaN;
        meanSquaredInnovation = 1.0;
        sampleCount = 0;
    }

    /** Records a Layer 2 innovation in sigma units and returns current vision-only trust. */
    public TrustResult observe(double timestamp_s, double innovationSigma) {
        if (!Double.isFinite(innovationSigma)) {
            return result();
        }

        if (Double.isNaN(lastTimestamp_s)) {
            lastTimestamp_s = timestamp_s;
            meanSquaredInnovation = innovationSigma * innovationSigma;
            sampleCount = 1;
            return result();
        }
        if (timestamp_s <= lastTimestamp_s) {
            return result();
        }

        double alpha = 1.0 - Math.exp(-(timestamp_s - lastTimestamp_s) / timeConstant_s);
        meanSquaredInnovation += alpha * (innovationSigma * innovationSigma - meanSquaredInnovation);
        lastTimestamp_s = timestamp_s;
        sampleCount++;
        return result();
    }

    private TrustResult result() {
        double jitterSigma = Math.sqrt(meanSquaredInnovation);
        if (sampleCount < warmupSamples) {
            return new TrustResult(1.0, jitterSigma, false);
        }

        // 1 sigma is nominal consistency. Stable vision can become slightly more influential;
        // jittery vision is progressively de-weighted, with conservative hard limits.
        double multiplier = 0.70 + 0.30 * jitterSigma;
        multiplier = Math.max(minMultiplier, Math.min(maxMultiplier, multiplier));
        return new TrustResult(multiplier, jitterSigma, true);
    }
}
