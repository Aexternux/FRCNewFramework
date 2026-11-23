package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.Util;

/**
 * Follows a target pose with smooth motion.
 *
 * Uses P-control capped at maxSpeed for translation, with rotation gain normalized
 * against max angular velocity. Matches the frc2k26 follower exactly.
 */
public class PoseFollower {

    private double translate_kP = 3.0;
    private double rotate_kP = 4.0;

    private double maxSpeed_mps = Drive.MAX_LINEAR_VEL_mps;
    private Pose2d targetPose = new Pose2d();
    private boolean preserveMomentum;

    private static final double DEFAULT_MAX_ACCEL_mps2 = Drive.MAX_AUTO_FORWARD_ACC_mps2;
    private static final double DT_s = 0.02;

    private double maxAccel_mps2 = 200;

    private static final double STOP_DISTANCE_m = 0.10;

    private double lastVx_mps = 0.0;
    private double lastVy_mps = 0.0;
    private double lastOmega_radps = 0.0;

    public PoseFollower(Pose2d targetPose, double maxSpeed_mps, double translation_kP,
            double rotation_kP) {
        setParams(targetPose, maxSpeed_mps, translation_kP, rotation_kP);
    }

    public PoseFollower(Pose2d targetPose, double maxSpeed_mps) {
        this.targetPose = targetPose;
        this.maxSpeed_mps = maxSpeed_mps;
    }

    public void setParams(Pose2d targetPose, double maxSpeed_mps) {
        setParams(targetPose, maxSpeed_mps, translate_kP, rotate_kP);
    }

    public void setParams(Pose2d targetPose, double maxSpeed_mps, double translation_kP,
            double rotation_kP) {
        setParams(targetPose, maxSpeed_mps, translation_kP, rotation_kP, false);
    }

    /**
     * When preserving momentum across continuous actions, never command below the last
     * loop's speed so the robot carries through action boundaries instead of dipping.
     * Cleared by reset() whenever a non-preserving target takes over.
     */
    public void setParams(Pose2d targetPose, double maxSpeed_mps, double translation_kP,
            double rotation_kP, boolean preserveMomentum) {
        this.targetPose = targetPose;
        this.maxSpeed_mps = Math.max(0.0, maxSpeed_mps);
        this.translate_kP = Math.max(0.0, translation_kP);
        this.rotate_kP = Math.max(0.0, rotation_kP);
        this.preserveMomentum = preserveMomentum;
    }

    /**
     * Reset the velocity smoothing (call when starting a new path)
     */
    public void reset() {
        lastVx_mps = 0.0;
        lastVy_mps = 0.0;
        lastOmega_radps = 0.0;
    }

    public void setMaxAccel(double maxAccel_mps2) {
        this.maxAccel_mps2 = maxAccel_mps2;
    }

    public void resetMaxAccel() {
        this.maxAccel_mps2 = DEFAULT_MAX_ACCEL_mps2;
    }

    public ChassisSpeeds process(Pose2d currentPose) {
        Transform2d poseDiff = targetPose.minus(currentPose);

        double distance_m = Math.hypot(poseDiff.getX(), poseDiff.getY());

        Rotation2d rotationDiff = poseDiff.getRotation();
        double radDiff = MathUtil.angleModulus(rotationDiff.getRadians());
        double desiredOmega_radps = MathUtil.clamp(
                rotate_kP * radDiff * Drive.MAX_ANGULAR_VEL_radps,
                -Drive.MAX_ANGULAR_VEL_radps, Drive.MAX_ANGULAR_VEL_radps);

        double desiredVx_mps = 0.0;
        double desiredVy_mps = 0.0;

        if (distance_m > STOP_DISTANCE_m) {
            double dirX = poseDiff.getX() / distance_m;
            double dirY = poseDiff.getY() / distance_m;

            double speed_mps = Util.limit(translate_kP * distance_m, maxSpeed_mps);
            if (preserveMomentum) {
                speed_mps = Math.min(maxSpeed_mps,
                        Math.max(speed_mps, Math.hypot(lastVx_mps, lastVy_mps)));
            }

            desiredVx_mps = dirX * speed_mps;
            desiredVy_mps = dirY * speed_mps;
        }

        double maxLinearDelta = maxAccel_mps2 * DT_s;

        double vx_mps = rampValue(lastVx_mps, desiredVx_mps, maxLinearDelta);
        double vy_mps = rampValue(lastVy_mps, desiredVy_mps, maxLinearDelta);
        double omega_radps = desiredOmega_radps;

        lastVx_mps = vx_mps;
        lastVy_mps = vy_mps;
        lastOmega_radps = omega_radps;

        return new ChassisSpeeds(vx_mps, vy_mps, omega_radps);
    }

    private double rampValue(double current, double target, double maxDelta) {
        double diff = target - current;
        if (Math.abs(diff) <= maxDelta) {
            return target;
        }
        return current + Math.signum(diff) * maxDelta;
    }
}
