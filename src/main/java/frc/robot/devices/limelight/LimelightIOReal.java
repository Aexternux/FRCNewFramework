package frc.robot.devices.limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs.PoseObservation;
import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs.PoseType;

/** Real Limelight implementation backed by {@link LimelightHelpers} over NetworkTables. */
public class LimelightIOReal implements LimelightIO {

    private final String name;
    private final double mt1MaxDistance_m;

    public LimelightIOReal(LimelightConfig config) {
        this.name = config.name;
        this.mt1MaxDistance_m = config.mt1MaxDistance_m;
    }

    @Override
    public void updateInputs(LimelightIOInputs inputs) {
        inputs.connected = LimelightHelpers.getLatestResults(name) != null;

        if (!inputs.connected) {
            inputs.hasTarget = false;
            inputs.primaryTagId = -1;
            inputs.visibleTagIds = new int[0];
            inputs.targetSpaceX_m = 0.0;
            inputs.targetSpaceZ_m = 0.0;
            inputs.poseObservations = new PoseObservation[0];
            return;
        }

        // Tracking
        inputs.hasTarget = LimelightHelpers.getTV(name);
        inputs.tx = LimelightHelpers.getTX(name);
        inputs.ty = LimelightHelpers.getTY(name);
        inputs.ta = LimelightHelpers.getTA(name);
        inputs.primaryTagId = (int) LimelightHelpers.getFiducialID(name);

        // Best target pose in camera space: [x, y, z, roll, pitch, yaw]. z is forward distance.
        double[] targetSpace = LimelightHelpers.getTargetPose_CameraSpace(name);
        if (targetSpace != null && targetSpace.length >= 3) {
            inputs.targetSpaceX_m = targetSpace[0];
            inputs.targetSpaceZ_m = targetSpace[2];
        } else {
            inputs.targetSpaceX_m = 0.0;
            inputs.targetSpaceZ_m = 0.0;
        }

        // Pose estimation: prefer MegaTag1 when very close, otherwise MegaTag2 (gyro-fused).
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        inputs.visibleTagIds = tagIds(mt1);

        PoseObservation observation = null;
        if (mt1 != null && mt1.tagCount > 0 && mt1.avgTagDist < mt1MaxDistance_m) {
            observation = new PoseObservation(
                    mt1.timestampSeconds,
                    new Pose3d(mt1.pose),
                    maxAmbiguity(mt1),
                    mt1.tagCount,
                    mt1.avgTagDist,
                    PoseType.MEGATAG_1);
        } else {
            LimelightHelpers.PoseEstimate mt2 =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
            if (mt2 != null && mt2.tagCount > 0) {
                observation = new PoseObservation(
                        mt2.timestampSeconds,
                        new Pose3d(mt2.pose),
                        0.0,
                        mt2.tagCount,
                        mt2.avgTagDist,
                        PoseType.MEGATAG_2);
            }
        }

        inputs.poseObservations = observation == null
                ? new PoseObservation[0]
                : new PoseObservation[] {observation};

        if (observation != null) {
            Logger.recordOutput("Vision/Limelight/PoseEstimate", observation.pose());
        }
    }

    private static int[] tagIds(LimelightHelpers.PoseEstimate estimate) {
        if (estimate == null || estimate.rawFiducials == null) {
            return new int[0];
        }
        int[] ids = new int[estimate.rawFiducials.length];
        for (int i = 0; i < ids.length; i++) {
            ids[i] = estimate.rawFiducials[i].id;
        }
        return ids;
    }

    private static double maxAmbiguity(LimelightHelpers.PoseEstimate estimate) {
        if (estimate.rawFiducials == null) {
            return 0.0;
        }
        double worst = 0.0;
        for (LimelightHelpers.RawFiducial fiducial : estimate.rawFiducials) {
            worst = Math.max(worst, fiducial.ambiguity);
        }
        return worst;
    }

    @Override
    public void setPipeline(int index) {
        LimelightHelpers.setPipelineIndex(name, index);
    }

    @Override
    public void setFiducialFilters(int[] ids) {
        LimelightHelpers.SetFiducialIDFiltersOverride(name, ids);
    }

    @Override
    public void setRobotOrientation(Rotation2d heading) {
        LimelightHelpers.SetRobotOrientation(name, heading.getDegrees(), 0, 0, 0, 0, 0);
    }
}
