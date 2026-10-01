package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Robot-relative pose controller for short autonomous moves and driver assists. */
public class PoseFollower {
    private static final double TRANSLATION_DEADBAND_m = 0.03;
    private static final double ROTATION_DEADBAND_rad = Math.toRadians(1.5);

    private double translation_kP = 3.0;
    private double rotation_kP = 4.0;
    private double maxSpeed_mps = Drive.MAX_LINEAR_VEL_mps;
    private Pose2d targetPose = new Pose2d();

    public PoseFollower(Pose2d targetPose, double maxSpeed_mps, double translation_kP,
            double rotation_kP) {
        setParams(targetPose, maxSpeed_mps, translation_kP, rotation_kP);
    }

    public PoseFollower(Pose2d targetPose, double maxSpeed_mps) {
        setParams(targetPose, maxSpeed_mps, translation_kP, rotation_kP);
    }

    public void setParams(Pose2d targetPose, double maxSpeed_mps) {
        setParams(targetPose, maxSpeed_mps, translation_kP, rotation_kP);
    }

    public void setParams(Pose2d targetPose, double maxSpeed_mps, double translation_kP,
            double rotation_kP) {
        setParams(targetPose, maxSpeed_mps, translation_kP, rotation_kP, false);
    }

    /**
     * Momentum is maintained by Drive's chassis acceleration limiter, not a second hidden limiter
     * in this class. The flag remains so ActionList retains its explicit handoff semantics.
     */
    public void setParams(Pose2d targetPose, double maxSpeed_mps, double translation_kP,
            double rotation_kP, boolean preserveMomentum) {
        this.targetPose = targetPose;
        this.maxSpeed_mps = Math.max(0.0, maxSpeed_mps);
        this.translation_kP = Math.max(0.0, translation_kP);
        this.rotation_kP = Math.max(0.0, rotation_kP);
    }

    /** Drive owns acceleration limiting, so this controller has no velocity state to reset. */
    public void reset() {}

    /** Returns a bounded robot-relative velocity request toward the target pose. */
    public ChassisSpeeds process(Pose2d currentPose) {
        Transform2d error = targetPose.minus(currentPose);
        double errorX_m = error.getX();
        double errorY_m = error.getY();
        double distance_m = Math.hypot(errorX_m, errorY_m);

        double vx_mps = 0.0;
        double vy_mps = 0.0;
        if (distance_m > TRANSLATION_DEADBAND_m) {
            // Scale both axes together: P control decelerates near the target without changing the
            // direction of travel, and the requested vector cannot exceed maxSpeed.
            double scale = Math.min(translation_kP, maxSpeed_mps / distance_m);
            vx_mps = errorX_m * scale;
            vy_mps = errorY_m * scale;
        }

        double rotationError_rad = error.getRotation().getRadians();
        double omega_radps = Math.abs(rotationError_rad) <= ROTATION_DEADBAND_rad
                ? 0.0
                : MathUtil.clamp(rotation_kP * rotationError_rad,
                        -Drive.MAX_ANGULAR_VEL_radps, Drive.MAX_ANGULAR_VEL_radps);

        return new ChassisSpeeds(vx_mps, vy_mps, omega_radps);
    }
}
