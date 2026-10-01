package frc.robot.devices.limelight;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.radiansToDegrees;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs.PoseObservation;
import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs.PoseType;

/**
 * Simulated Limelight. Rather than running a full camera simulation, it synthesizes pose
 * observations geometrically from the known (ground-truth) robot pose and the AprilTag field
 * layout: any tag within range and the camera FOV produces a perfect pose observation of the true
 * robot pose. This is deterministic and dependency-free.
 */
public class LimelightIOSim implements LimelightIO {

    // Camera model used to decide which tags are "seen".
    private static final double FOV_HORIZONTAL_rad = degreesToRadians(59.6);
    private static final double FOV_VERTICAL_rad = degreesToRadians(49.7);
    private static final double MAX_TAG_DISTANCE_m = 3.5;
    private static final double MIN_TAG_DISTANCE_m = 0.5;

    private final Transform3d robotToCamera;
    private final double mt1MaxDistance_m;
    private final AprilTagFieldLayout layout;

    private Pose2d robotPose = null;

    public LimelightIOSim(LimelightConfig config) {
        this.robotToCamera = config.robotToCamera;
        this.mt1MaxDistance_m = config.mt1MaxDistance_m;
        this.layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    @Override
    public void updateSimPose(Pose2d robotPose) {
        this.robotPose = robotPose;
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        inputs.connected = true;

        if (robotPose == null || layout == null) {
            noTarget(inputs);
            return;
        }

        Pose3d cameraPose = new Pose3d(robotPose).plus(robotToCamera);

        List<Integer> visibleTags = new ArrayList<>();
        double bestDistance = Double.MAX_VALUE;
        Pose3d bestTagPose = null;
        int bestTagId = -1;

        for (var tag : layout.getTags()) {
            Transform3d cameraToTag = new Transform3d(cameraPose, tag.pose);
            double distance = cameraToTag.getTranslation().getNorm();
            if (distance > MAX_TAG_DISTANCE_m || distance < MIN_TAG_DISTANCE_m) {
                continue;
            }
            // Tag must be in front of the camera, and the camera in front of the tag's face
            // (AprilTags are only visible from the front).
            if (cameraToTag.getX() <= 0) {
                continue;
            }
            if (new Transform3d(tag.pose, cameraPose).getX() <= 0) {
                continue;
            }

            double yaw = Math.atan2(cameraToTag.getY(), cameraToTag.getX());
            double pitch = Math.atan2(cameraToTag.getZ(), cameraToTag.getX());
            if (Math.abs(yaw) >= FOV_HORIZONTAL_rad / 2 || Math.abs(pitch) >= FOV_VERTICAL_rad / 2) {
                continue;
            }

            visibleTags.add(tag.ID);
            if (distance < bestDistance) {
                bestDistance = distance;
                bestTagPose = tag.pose;
                bestTagId = tag.ID;
            }
        }

        if (visibleTags.isEmpty() || bestTagPose == null) {
            noTarget(inputs);
            return;
        }

        Transform3d cameraToBest = new Transform3d(cameraPose, bestTagPose);
        double yaw = Math.atan2(cameraToBest.getY(), cameraToBest.getX());
        double pitch = Math.atan2(cameraToBest.getZ(), cameraToBest.getX());

        inputs.hasTarget = true;
        inputs.tx = radiansToDegrees(-yaw);
        inputs.ty = radiansToDegrees(pitch);
        inputs.ta = 0.0;
        inputs.primaryTagId = bestTagId;
        inputs.visibleTagIds = visibleTags.stream().mapToInt(Integer::intValue).toArray();

        // Camera-space target pose: z forward, x lateral (right positive), matching the Limelight
        // targetpose_cameraspace convention that the tracking subsystem consumes.
        inputs.targetSpaceZ_m = cameraToBest.getX();
        inputs.targetSpaceX_m = -cameraToBest.getY();

        PoseType type = bestDistance < mt1MaxDistance_m ? PoseType.MEGATAG_1 : PoseType.MEGATAG_2;
        inputs.poseObservations = new PoseObservation[] {
            new PoseObservation(
                    Timer.getFPGATimestamp(),
                    new Pose3d(robotPose),
                    0.0,
                    visibleTags.size(),
                    bestDistance,
                    type)
        };

        logSightLines(cameraPose, visibleTags);
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
        Logger.recordOutput("Vision/Limelight/SightLines", new Pose3d[0]);
    }

    private void logSightLines(Pose3d cameraPose, List<Integer> visibleTags) {
        List<Pose3d> sightLines = new ArrayList<>();
        for (int id : visibleTags) {
            layout.getTagPose(id).ifPresent(tagPose -> {
                sightLines.add(cameraPose);
                sightLines.add(tagPose);
            });
        }
        Logger.recordOutput("Vision/Limelight/SightLines", sightLines.toArray(new Pose3d[0]));
    }
}
