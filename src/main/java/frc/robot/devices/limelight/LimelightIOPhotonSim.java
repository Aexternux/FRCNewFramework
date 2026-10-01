package frc.robot.devices.limelight;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs.PoseObservation;
import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs.PoseType;

/**
 * Realistic simulated Limelight backed by PhotonVision's {@link VisionSystemSim}. Unlike
 * {@link LimelightIOSim} (which hands back a perfect copy of the pose it is given), this drives a
 * simulated camera against a ground-truth pose and produces noisy, latency-delayed AprilTag
 * detections. The multi-tag PnP result is converted back into the {@link PoseObservation} records
 * the rest of the vision stack already consumes, so nothing downstream changes.
 */
public class LimelightIOPhotonSim implements LimelightIO {

    private final VisionSystemSim visionSim;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final Transform3d robotToCamera;
    private final double mt1MaxDistance_m;
    private final double maxSightRange_m;
    private final AprilTagFieldLayout layout;

    public LimelightIOPhotonSim(LimelightConfig config) {
        this.robotToCamera = config.robotToCamera;
        this.mt1MaxDistance_m = config.mt1MaxDistance_m;
        this.maxSightRange_m = config.maxSightRange_m;
        this.layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        visionSim = new VisionSystemSim(config.name);
        visionSim.addAprilTags(layout);

        // Camera properties matched to the frc2k26 VisionIOSimPhoton setup that worked well:
        // high frame rate + low latency + no injected pixel noise keeps the estimate smooth.
        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(1280, 800, Rotation2d.fromDegrees(70.0));
        props.setFPS(90.0);
        props.setAvgLatencyMs(11.0);
        props.setLatencyStdDevMs(2.0);

        camera = new PhotonCamera(config.name);
        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, props);
        cameraSim.enableDrawWireframe(true);

        // Cap how far the fake camera can see, if the config asked for it (sim short-range camera).
        if (Double.isFinite(config.maxSightRange_m)) {
            cameraSim.setMaxSightRange(config.maxSightRange_m);
        }
        visionSim.addCamera(cameraSim, robotToCamera);

        poseEstimator = new PhotonPoseEstimator(
                layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateSimPose(Pose2d robotPose) {
        if (robotPose != null) {
            visionSim.update(robotPose);
        }
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        inputs.connected = true;

        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            noTarget(inputs);
            return;
        }
        PhotonPipelineResult result = results.get(results.size() - 1);

        if (!result.hasTargets()) {
            noTarget(inputs);
            return;
        }

        // Drop any detections past our sight range so "seen" means the same thing everywhere.
        List<PhotonTrackedTarget> targets = new ArrayList<>();
        for (PhotonTrackedTarget t : result.getTargets()) {
            if (t.getBestCameraToTarget().getTranslation().getNorm() <= maxSightRange_m) {
                targets.add(t);
            }
        }
        if (targets.isEmpty()) {
            noTarget(inputs);
            return;
        }

        // Real Limelights pick a "primary" tag; the sim doesn't, so we pick the closest ourselves.
        PhotonTrackedTarget best = closestTarget(targets);
        inputs.hasTarget = true;
        inputs.tx = -best.getYaw();  // PhotonVision yaw is CCW+; Limelight tx is right+
        inputs.ty = best.getPitch();
        inputs.ta = best.getArea();
        inputs.primaryTagId = best.getFiducialId();

        Transform3d camToTarget = best.getBestCameraToTarget();
        inputs.targetSpaceZ_m = camToTarget.getX();   // forward distance
        inputs.targetSpaceX_m = -camToTarget.getY();  // lateral (right positive)

        int[] ids = new int[targets.size()];
        for (int i = 0; i < ids.length; i++) {
            ids[i] = targets.get(i).getFiducialId();
        }
        inputs.visibleTagIds = ids;

        // ---- Pose estimation via multi-tag PnP ----
        Optional<org.photonvision.EstimatedRobotPose> estimate = poseEstimator.update(result);
        if (estimate.isPresent()) {
            Pose3d pose = estimate.get().estimatedPose;
            List<PhotonTrackedTarget> used = estimate.get().targetsUsed;

            double totalDist = 0.0;
            double worstAmbiguity = 0.0;
            for (PhotonTrackedTarget t : used) {
                totalDist += t.getBestCameraToTarget().getTranslation().getNorm();
                worstAmbiguity = Math.max(worstAmbiguity, t.getPoseAmbiguity());
            }
            double avgDist = used.isEmpty() ? 0.0 : totalDist / used.size();

            PoseObservation observation = new PoseObservation(
                    estimate.get().timestampSeconds,
                    pose,
                    worstAmbiguity,
                    used.size(),
                    avgDist,
                    avgDist < mt1MaxDistance_m ? PoseType.MEGATAG_1 : PoseType.MEGATAG_1);
            inputs.poseObservations = new PoseObservation[] {observation};
            Logger.recordOutput("Vision/PhotonSim/PoseEstimate", pose);
        } else {
            inputs.poseObservations = new PoseObservation[0];
        }
    }

    // The nearest tag to the camera (smallest camera-to-target distance).
    private static PhotonTrackedTarget closestTarget(List<PhotonTrackedTarget> targets) {
        PhotonTrackedTarget closest = targets.get(0);
        double closestDist_m = closest.getBestCameraToTarget().getTranslation().getNorm();
        for (PhotonTrackedTarget t : targets) {
            double dist_m = t.getBestCameraToTarget().getTranslation().getNorm();
            if (dist_m < closestDist_m) {
                closest = t;
                closestDist_m = dist_m;
            }
        }
        return closest;
    }

    private void noTarget(LimelightIOInputs inputs) {
        inputs.hasTarget = false;
        inputs.tx = 0.0;
        inputs.ty = 0.0;
        inputs.ta = 0.0;
        inputs.primaryTagId = -1;
        inputs.visibleTagIds = new int[0];
        inputs.targetSpaceX_m = 0.0;
        inputs.targetSpaceZ_m = 0.0;
        inputs.poseObservations = new PoseObservation[0];
    }
}
