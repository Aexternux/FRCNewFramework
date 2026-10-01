package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.devices.limelight.Limelight;
import frc.robot.devices.limelight.LimelightConfig;
import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs.PoseObservation;
import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs.PoseType;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionFusion.Measurement;
import frc.robot.subsystems.vision.PoseKalmanFilter.GateResult;

public class Vision extends SubsystemBase<Vision.Command> {

    public enum Command {
        DISABLED,
        ENABLED
    }

    private static Vision instance;

    private final List<Limelight> limelights = new ArrayList<>();
    private final AprilTagFieldLayout tagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private final Drive drive;
    private final PoseKalmanFilter filter = new PoseKalmanFilter(
            VisionConstants.GATE_PROCESS_STD_TRANSLATION_mPerRootS,
            VisionConstants.GATE_PROCESS_STD_ROTATION_radPerRootS);
    private final VisionTrustTracker trustTracker = new VisionTrustTracker(
            VisionConstants.TRUST_TIME_CONSTANT_s, VisionConstants.TRUST_WARMUP_SAMPLES,
            VisionConstants.TRUST_MIN_STD_DEV_MULTIPLIER,
            VisionConstants.TRUST_MAX_STD_DEV_MULTIPLIER);

    private Pose2d fusedPose = null;
    private Pose2d filteredPose = null;
    private int acceptedThisCycle = 0;
    private int gateRejectedThisCycle = 0;
    private int forwardedThisCycle = 0;
    private double adaptiveStdDevMultiplier = 1.0;
    private double visionJitterSigma = 0.0;
    private boolean trustWarmedUp = false;
    private double rejectObservationsBefore_s = Double.NEGATIVE_INFINITY;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
            System.out.println("Vision initialized.");
        }
        return instance;
    }

    private Vision() {
        super("Vision");
        for (VisionConstants.CameraConfig camera : VisionConstants.CAMERAS) {
            limelights.add(new Limelight(
                    "Vision/" + camera.name(),
                    new LimelightConfig(camera.name())
                            .withRobotToCamera(camera.robotToCamera())
                            .withMaxSightRange(VisionConstants.SIM_MAX_SIGHT_RANGE_m)));
        }
        drive = Drive.getInstance();
        setCommand(Command.DISABLED);
    }

    @Override
    protected void inputPeriodic() {
        for (Limelight limelight : limelights) {
            limelight.setRobotOrientation(drive.getRotation());
            limelight.updateSimPose(drive.getPose());
            limelight.readInputs();
        }
    }

    @Override
    protected void handle() {
        fusedPose = null;
        filteredPose = null;
        acceptedThisCycle = 0;
        gateRejectedThisCycle = 0;
        forwardedThisCycle = 0;

        // Layer 1: only fuse camera frames captured at nearly the same instant. Measurements from
        // different time windows retain their own timestamps and are handed to WPILib separately.
        List<Measurement> fusedMeasurements = fuseSynchronousMeasurements(collectMeasurements());
        if (fusedMeasurements.isEmpty() || getCommand() == Command.DISABLED) {
            filteredPose = filter.isInitialized() ? filter.getEstimate() : null;
            return;
        }

        for (Measurement fused : fusedMeasurements) {
            fusedPose = fused.pose();
            double stdX = Math.sqrt(fused.varX());
            double stdY = Math.sqrt(fused.varY());
            double stdTheta = Math.sqrt(fused.varTheta());

            // Layer 2 is vision-only. It may reject or de-weight a dubious camera measurement,
            // but it never creates a second drivetrain pose estimate.
            GateResult gate = filter.evaluateAndUpdate(
                    fused.pose(), fused.timestamp(), stdX, stdY, stdTheta,
                    VisionConstants.GATE_MAX_INNOVATION_SIGMA,
                    VisionConstants.GATE_MAX_STD_DEV_MULTIPLIER);
            VisionTrustTracker.TrustResult trust =
                    trustTracker.observe(fused.timestamp(), gate.innovationSigma());
            adaptiveStdDevMultiplier = trust.stdDevMultiplier();
            visionJitterSigma = trust.jitterSigma();
            trustWarmedUp = trust.warmedUp();
            if (!gate.accepted()) {
                gateRejectedThisCycle++;
                continue;
            }

            // Layer 3 adapts only covariance based on recent vision consistency. It contributes
            // no pose or odometry information of its own.
            double multiplier = gate.stdDevMultiplier() * trust.stdDevMultiplier();
            drive.addVisionMeasurement(fused.pose(), fused.timestamp(), VecBuilder.fill(
                    stdX * multiplier, stdY * multiplier, stdTheta * multiplier));
            forwardedThisCycle++;
        }

        filteredPose = filter.isInitialized() ? filter.getEstimate() : null;
    }

    @Override
    protected void outputPeriodic() {
        Logger.recordOutput("Vision/AcceptedObservations", acceptedThisCycle);
        Logger.recordOutput("Vision/GateRejectedObservations", gateRejectedThisCycle);
        Logger.recordOutput("Vision/ForwardedMeasurements", forwardedThisCycle);
        Logger.recordOutput("Vision/Trust/StdDevMultiplier", adaptiveStdDevMultiplier);
        Logger.recordOutput("Vision/Trust/JitterSigma", visionJitterSigma);
        Logger.recordOutput("Vision/Trust/WarmedUp", trustWarmedUp);
        Logger.recordOutput("Vision/HasFusedPose", fusedPose != null);
        if (fusedPose != null) {
            Logger.recordOutput("Vision/FusedPose", fusedPose);
        }
        if (filteredPose != null) {
            Logger.recordOutput("Vision/FilteredPose", filteredPose);
        }
        logSightlines();
    }

    // Log lines from each camera to the tags it sees, so you can watch what vision is looking at in
    // AdvantageScope (add the "Sightlines" fields to a 3D field view).
    private void logSightlines() {
        Pose3d robot = new Pose3d(drive.getPose());

        List<Pose3d> allTagPoses = new ArrayList<>();   // every visible tag, all cameras.
        List<Pose3d> allSightlines = new ArrayList<>(); // camera->tag line segments, all cameras.
        for (Limelight limelight : limelights) {
            Pose3d camera = robot.transformBy(limelight.getRobotToCamera()); // where this lens is.
            List<Pose3d> tagPoses = new ArrayList<>();
            List<Pose3d> sightlines = new ArrayList<>();
            for (int tagId : limelight.getVisibleTagIds()) {
                tagLayout.getTagPose(tagId).ifPresent(tagPose -> {
                    tagPoses.add(tagPose);
                    // A line = two points in a row: camera, then the tag.
                    sightlines.add(camera);
                    sightlines.add(tagPose);
                });
            }
            Logger.recordOutput(limelight.getName() + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(limelight.getName() + "/Sightlines", sightlines.toArray(new Pose3d[0]));
            allTagPoses.addAll(tagPoses);
            allSightlines.addAll(sightlines);
        }
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/Sightlines", allSightlines.toArray(new Pose3d[0]));
    }

    // ---- Layer 1: build per-limelight measurements ----

    private List<Measurement> collectMeasurements() {
        List<Measurement> measurements = new ArrayList<>();
        for (Limelight limelight : limelights) {
            for (PoseObservation observation : limelight.getPoseObservations()) {
                if (observation.timestamp() < rejectObservationsBefore_s) {
                    continue;
                }
                if (shouldReject(observation)) {
                    continue;
                }
                measurements.add(toMeasurement(observation));
                acceptedThisCycle++;
            }
        }
        return measurements;
    }

    /** Fuses only measurements captured within {@link VisionConstants#FUSION_TIMESTAMP_WINDOW_s}. */
    private List<Measurement> fuseSynchronousMeasurements(List<Measurement> measurements) {
        measurements.sort(Comparator.comparingDouble(Measurement::timestamp));
        List<Measurement> fusedMeasurements = new ArrayList<>();
        List<Measurement> cluster = new ArrayList<>();
        double clusterStart_s = Double.NaN;

        for (Measurement measurement : measurements) {
            if (cluster.isEmpty()) {
                clusterStart_s = measurement.timestamp();
            } else if (measurement.timestamp() - clusterStart_s
                    > VisionConstants.FUSION_TIMESTAMP_WINDOW_s) {
                fusedMeasurements.add(VisionFusion.fuse(cluster));
                cluster.clear();
                clusterStart_s = measurement.timestamp();
            }
            cluster.add(measurement);
        }
        if (!cluster.isEmpty()) {
            fusedMeasurements.add(VisionFusion.fuse(cluster));
        }
        return fusedMeasurements;
    }

    private boolean shouldReject(PoseObservation observation) {
        if (observation.tagCount() == 0) {
            return true;
        }
        if (observation.tagCount() == 1 && observation.ambiguity() > VisionConstants.MAX_AMBIGUITY) {
            return true;
        }
        if (Math.abs(observation.pose().getZ()) > VisionConstants.MAX_Z_ERROR_m) {
            return true;
        }
        Pose2d pose = observation.pose().toPose2d();
        return pose.getX() < 0.0 || pose.getY() < 0.0;
    }

    private Measurement toMeasurement(PoseObservation observation) {
        double factor = Math.pow(observation.avgTagDistance(), 2.0)
                / Math.max(observation.tagCount(), 1);

        double linearStd = VisionConstants.LINEAR_STD_BASELINE_m * factor;
        double rotationStd = observation.type() == PoseType.MEGATAG_2
                ? VisionConstants.MEGATAG2_ROTATION_STD_rad
                : VisionConstants.ANGULAR_STD_BASELINE_rad * factor;

        return new Measurement(
                observation.pose().toPose2d(),
                linearStd * linearStd,
                linearStd * linearStd,
                rotationStd * rotationStd,
                observation.timestamp());
    }

    // ---- Public API ----

    public void enable() {
        setCommand(Command.ENABLED);
    }

    public void disable() {
        setCommand(Command.DISABLED);
    }

    /**
     * Clears vision history and discards camera frames captured before an external odometry reset.
     * The next new camera observation establishes a new vision-only gate reference.
     */
    public void resetPose(Pose2d pose) {
        filter.clear();
        trustTracker.clear();
        fusedPose = null;
        filteredPose = null;
        adaptiveStdDevMultiplier = 1.0;
        visionJitterSigma = 0.0;
        trustWarmedUp = false;
        rejectObservationsBefore_s = Timer.getFPGATimestamp();
        Logger.recordOutput("Vision/ResetPose", pose);
    }

    /**
     * Tune how much legitimate motion the vision-only quality gate tolerates between frames.
     */
    public void setResponsiveness(double processStdTranslation_mPerRootS,
            double processStdRotation_radPerRootS) {
        filter.setProcessNoise(processStdTranslation_mPerRootS, processStdRotation_radPerRootS);
    }

    public boolean hasFusedPose() {
        return fusedPose != null;
    }

    /** The inverse-variance-fused limelight pose from this cycle, or null. */
    public Pose2d getFusedPose() {
        return fusedPose;
    }

    /** The Kalman-filtered pose from this cycle, or null before the filter initializes. */
    public Pose2d getFilteredPose() {
        return filteredPose;
    }
}
