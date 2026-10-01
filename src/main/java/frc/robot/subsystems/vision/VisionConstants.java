package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Tuning and camera layout for the {@link Vision} subsystem. */
public final class VisionConstants {

    private VisionConstants() {}

    /** A single configured limelight: NetworkTables name + mounting transform. */
    public static record CameraConfig(String name, Transform3d robotToCamera) {}

    /**
     * The cameras the {@link Vision} subsystem manages. Add as many limelights as you like by
     * appending entries here — each one becomes its own device and contributes to the fused pose.
     */
    // NOTE: robotToCamera is consumed only by the simulation, which uses it to decide which tags
    // fall in the camera FOV. It must reflect the real mounting. In WPILib's NWU frame a positive
    // pitch points the camera DOWN; an AprilTag camera is tilted UP, so the pitch is negative.
    public static final CameraConfig[] CAMERAS = {
        new CameraConfig(
                "limelight-sean",
                new Transform3d(
                        new Translation3d(0.0889, -0.2794, 0.4445),
                        new Rotation3d(0, Units.degreesToRadians(-20), 0))),
        // new CameraConfig("limelight-back",
        //         new Transform3d(new Translation3d(-0.1, 0.0, 0.4), new Rotation3d(0, 0, Math.PI))),
    };

    // ---- Per-limelight measurement noise ----
    // A camera's reported pose variance grows with distance^2 and shrinks with tag count:
    //     std = baseline * (avgTagDistance^2 / tagCount)
    public static final double LINEAR_STD_BASELINE_m = 0.02;
    public static final double ANGULAR_STD_BASELINE_rad = 0.06;

    /** MegaTag2 derives heading from the gyro, so its rotation is given a huge (untrusted) std. */
    public static final double MEGATAG2_ROTATION_STD_rad = 1.0e3;

    // ---- Observation rejection ----
    public static final double MAX_AMBIGUITY = 0.3;
    public static final double MAX_Z_ERROR_m = 0.75;

    /** Sim only: cameras can't detect tags farther than this. */
    public static final double SIM_MAX_SIGHT_RANGE_m = 5.0;

    // ---- Layer 2: vision-only quality gate ----
    // Process noise is in units per sqrt(second). It lets the gate tolerate legitimate robot
    // motion between camera frames without using wheel odometry as a second input.
    public static final double GATE_PROCESS_STD_TRANSLATION_mPerRootS = 0.70;
    public static final double GATE_PROCESS_STD_ROTATION_radPerRootS = Units.degreesToRadians(56.0);

    /** Reject a camera measurement if it differs from recent vision by more than this many sigma. */
    public static final double GATE_MAX_INNOVATION_SIGMA = 4.0;

    /** Near the gate threshold, inflate the measurement std dev by up to this multiplier. */
    public static final double GATE_MAX_STD_DEV_MULTIPLIER = 2.0;

    // ---- Layer 3: adaptive trust from recent vision consistency ----
    /** Time horizon over which vision jitter affects dynamic measurement trust. */
    public static final double TRUST_TIME_CONSTANT_s = 0.75;

    /** Do not tighten measurement uncertainty until this many fresh camera samples are observed. */
    public static final int TRUST_WARMUP_SAMPLES = 10;

    /** Hard limits on Layer 3's multiplier of Layer 1's camera standard deviations. */
    public static final double TRUST_MIN_STD_DEV_MULTIPLIER = 0.75;
    public static final double TRUST_MAX_STD_DEV_MULTIPLIER = 2.0;

    /** Only camera observations captured this close together may be fused into one measurement. */
    public static final double FUSION_TIMESTAMP_WINDOW_s = 0.020;
}
