package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Detects a hit from the IMU and rebuilds the x/y that wheel odometry lost while skidding.
 *
 * <p>A hit shows up as the Pigeon's measured acceleration disagreeing with the acceleration the
 * wheels report. When that happens, the estimate restarts from the pose a couple of loops before the
 * jolt and double-integrates the IMU until the wheels have traction again. Heading is never touched:
 * the gyro is still right after a hit, only the wheel odometry is not.
 */
class CollisionEstimator {
    private static final double ROBOT_MASS_kg = 56.7; // deploy/pathplanner/settings.json robotMass
    private static final double STANDARD_GRAVITY_mps2 = 9.80665;

    // IMU vs wheel acceleration disagreement that starts a hit, and the level that counts as settled.
    private static final double HIT_START_mps2 = 10.0;
    private static final double HIT_END_mps2 = 3.0;
    // Wheels must also agree with the integrated velocity again, or a steady shove never "ends".
    private static final double HIT_END_VEL_mps = 0.3;
    private static final double HIT_END_CALM_s = 0.10;
    // IMU double integration drifts, so a hit window is capped.
    private static final double MAX_HIT_WINDOW_s = 1.0;
    // Corrections outside this band are noise (too small) or a bad integration (too large).
    private static final double MIN_CORRECTION_m = 0.02;
    private static final double MAX_CORRECTION_m = 1.0;
    // The anchor is this many samples before the first jolted one, so it predates any skid.
    private static final int ANCHOR_LOOKBACK_SAMPLES = 2;
    private static final int HISTORY_SIZE = 8;
    // Slow tracking of IMU mount/tilt offset while there is no hit.
    private static final double BIAS_ALPHA = 0.02;
    private static final double NO_TIME_s = -1.0;

    private record Sample(double t_s, Translation2d position_m, Translation2d fieldVel_mps,
            Translation2d fieldAcc_mps2) {}

    private final String scope;
    private final ArrayDeque<Sample> history = new ArrayDeque<>();
    private Translation2d bias_mps2;
    private ChassisSpeeds lastSpeeds;
    private double lastUpdate_s = NO_TIME_s;

    private boolean inHit;
    private double hitStart_s;
    private double lastIntegrated_s;
    private double calmSince_s = NO_TIME_s;
    private double peakAcc_mps2;
    private Pose2d anchorPose = new Pose2d();
    private Translation2d anchorVel_mps = Translation2d.kZero;
    private Translation2d position_m = Translation2d.kZero;
    private Translation2d vel_mps = Translation2d.kZero;
    private int hitCount;

    CollisionEstimator(String parentScope) {
        scope = parentScope + "/Collision";
    }

    /** Forgets history and any hit in progress. Call whenever the pose is reset externally. */
    void reset() {
        history.clear();
        lastSpeeds = null;
        lastUpdate_s = NO_TIME_s;
        inHit = false;
        calmSince_s = NO_TIME_s;
    }

    boolean isInHit() {
        return inHit;
    }

    /**
     * Feeds one loop of data.
     *
     * @param robotSpeeds measured robot-relative chassis speeds from the wheels
     * @param robotImuAcc_mps2 measured robot-relative IMU acceleration
     * @param detect false (e.g. while disabled) to only track the IMU bias
     * @return the corrected field x/y when a hit just ended and should be applied, otherwise null
     */
    Translation2d update(double now_s, Pose2d pose, ChassisSpeeds robotSpeeds,
            Translation2d robotImuAcc_mps2, boolean detect) {
        double dt_s = lastUpdate_s < 0.0 ? 0.0 : now_s - lastUpdate_s;
        Translation2d wheelAcc_mps2 = (lastSpeeds == null || dt_s <= 0.0) ? Translation2d.kZero
                : new Translation2d(
                        (robotSpeeds.vxMetersPerSecond - lastSpeeds.vxMetersPerSecond) / dt_s,
                        (robotSpeeds.vyMetersPerSecond - lastSpeeds.vyMetersPerSecond) / dt_s);
        lastSpeeds = robotSpeeds;
        lastUpdate_s = now_s;

        Translation2d rawOffset_mps2 = robotImuAcc_mps2.minus(wheelAcc_mps2);
        if (bias_mps2 == null) {
            bias_mps2 = rawOffset_mps2;
        }
        Translation2d imuAcc_mps2 = robotImuAcc_mps2.minus(bias_mps2);
        double disagreement_mps2 = imuAcc_mps2.minus(wheelAcc_mps2).getNorm();

        Rotation2d heading = pose.getRotation();
        Sample sample = new Sample(now_s, pose.getTranslation(),
                new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                        .rotateBy(heading),
                imuAcc_mps2.rotateBy(heading));
        history.addLast(sample);
        while (history.size() > HISTORY_SIZE) {
            history.removeFirst();
        }

        Translation2d corrected = null;
        if (!detect) {
            inHit = false;
            bias_mps2 = blendBias(rawOffset_mps2);
        } else if (!inHit) {
            if (disagreement_mps2 > HIT_START_mps2 && history.size() > ANCHOR_LOOKBACK_SAMPLES + 1) {
                startHit();
            } else {
                bias_mps2 = blendBias(rawOffset_mps2);
            }
        } else {
            integrate(sample);
            boolean calm = disagreement_mps2 < HIT_END_mps2
                    && vel_mps.getDistance(sample.fieldVel_mps()) < HIT_END_VEL_mps;
            if (!calm) {
                calmSince_s = NO_TIME_s;
            } else if (calmSince_s < 0.0) {
                calmSince_s = now_s;
            }
            boolean settled = calmSince_s >= 0.0 && now_s - calmSince_s >= HIT_END_CALM_s;
            boolean timedOut = now_s - hitStart_s >= MAX_HIT_WINDOW_s;
            if (settled || timedOut) {
                corrected = finishHit(pose, timedOut);
            }
        }

        Logger.recordOutput(scope + "/Active", inHit);
        Logger.recordOutput(scope + "/Disagreement_mps2", disagreement_mps2);
        Logger.recordOutput(scope + "/ImuAcc_mps2", new double[] {imuAcc_mps2.getX(), imuAcc_mps2.getY()});
        Logger.recordOutput(scope + "/WheelAcc_mps2", new double[] {wheelAcc_mps2.getX(), wheelAcc_mps2.getY()});
        Logger.recordOutput(scope + "/Bias_mps2", new double[] {bias_mps2.getX(), bias_mps2.getY()});
        if (inHit) {
            Logger.recordOutput(scope + "/ImuPose", new Pose2d(position_m, heading));
        }
        return corrected;
    }

    private Translation2d blendBias(Translation2d rawOffset_mps2) {
        return bias_mps2.times(1.0 - BIAS_ALPHA).plus(rawOffset_mps2.times(BIAS_ALPHA));
    }

    /** Anchors on a pre-jolt sample and catches the integration up to the current loop. */
    private void startHit() {
        List<Sample> samples = new ArrayList<>(history);
        int anchorIndex = samples.size() - 1 - ANCHOR_LOOKBACK_SAMPLES;
        Sample anchor = samples.get(anchorIndex);
        inHit = true;
        hitCount++;
        hitStart_s = anchor.t_s();
        lastIntegrated_s = anchor.t_s();
        calmSince_s = NO_TIME_s;
        peakAcc_mps2 = 0.0;
        anchorPose = new Pose2d(anchor.position_m(), Rotation2d.kZero);
        anchorVel_mps = anchor.fieldVel_mps();
        position_m = anchor.position_m();
        vel_mps = anchor.fieldVel_mps();
        for (int i = anchorIndex + 1; i < samples.size(); i++) {
            integrate(samples.get(i));
        }
        Logger.recordOutput(scope + "/HitCount", hitCount);
        Logger.recordOutput(scope + "/PreHitPose", anchorPose);
    }

    private void integrate(Sample sample) {
        double dt_s = sample.t_s() - lastIntegrated_s;
        if (dt_s <= 0.0) {
            return;
        }
        vel_mps = vel_mps.plus(sample.fieldAcc_mps2().times(dt_s));
        position_m = position_m.plus(vel_mps.times(dt_s));
        lastIntegrated_s = sample.t_s();
        peakAcc_mps2 = Math.max(peakAcc_mps2, sample.fieldAcc_mps2().getNorm());
    }

    private Translation2d finishHit(Pose2d pose, boolean timedOut) {
        inHit = false;
        double duration_s = lastIntegrated_s - hitStart_s;
        // Where the robot would be had it kept its pre-hit velocity: the gap is how far it was pushed.
        Translation2d coast_m = anchorPose.getTranslation().plus(anchorVel_mps.times(duration_s));
        Translation2d push_m = position_m.minus(coast_m);
        double correction_m = position_m.getDistance(pose.getTranslation());
        boolean apply = correction_m >= MIN_CORRECTION_m && correction_m <= MAX_CORRECTION_m;

        Logger.recordOutput(scope + "/Last/Duration_s", duration_s);
        Logger.recordOutput(scope + "/Last/TimedOut", timedOut);
        Logger.recordOutput(scope + "/Last/PeakAccel_g", peakAcc_mps2 / STANDARD_GRAVITY_mps2);
        Logger.recordOutput(scope + "/Last/PeakForce_N", ROBOT_MASS_kg * peakAcc_mps2);
        Logger.recordOutput(scope + "/Last/Impulse_Ns",
                ROBOT_MASS_kg * vel_mps.minus(anchorVel_mps).getNorm());
        Logger.recordOutput(scope + "/Last/Pushed_m", push_m.getNorm());
        Logger.recordOutput(scope + "/Last/PushDirection_deg",
                push_m.getNorm() > 0.0 ? push_m.getAngle().getDegrees() : 0.0);
        Logger.recordOutput(scope + "/Last/Correction_m", correction_m);
        Logger.recordOutput(scope + "/Last/Applied", apply);
        Logger.recordOutput(scope + "/Last/OdometryPose", pose);
        Logger.recordOutput(scope + "/Last/CorrectedPose", new Pose2d(position_m, pose.getRotation()));

        // History holds pre-correction poses; a later hit must anchor on corrected ones.
        history.clear();
        return apply ? position_m : null;
    }
}
