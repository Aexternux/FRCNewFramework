package frc.robot.devices.limelight;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.Constants;
import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs.PoseObservation;

/**
 * Limelight smart-camera device. Wraps the IO layer and exposes both tracking data (tx/ty/ta, tag
 * ids) and AprilTag pose observations. Consumed by the {@code Vision} subsystem.
 */
public class Limelight {

    public enum Mode {
        REAL,
        SIM
    }

    private final String name;
    private final LimelightIO io;
    private final Transform3d robotToCamera;
    private final LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();

    public Limelight(String name, LimelightConfig config, Mode mode) {
        this.name = name;
        this.robotToCamera = config.robotToCamera;
        this.io = mode == Mode.REAL ? new LimelightIOReal(config) : new LimelightIOPhotonSim(config);
    }

    public Limelight(String name, LimelightConfig config) {
        this(name, config, Constants.currentMode == Constants.Mode.REAL ? Mode.REAL : Mode.SIM);
    }

    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    // The name this camera logs under (also its NetworkTables key).
    public String getName() {
        return name;
    }

    // Where this camera sits on the robot (robot origin -> camera lens). Used for sightlines/sim.
    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }

    // ---- Tracking ----
    public boolean isConnected() {
        return inputs.connected;
    }

    public boolean hasTarget() {
        return inputs.hasTarget;
    }

    /** Horizontal offset to the best target, degrees (negative = left). */
    public double getTX() {
        return inputs.tx;
    }

    /** Vertical offset to the best target, degrees (negative = down). */
    public double getTY() {
        return inputs.ty;
    }

    /** Best target area, percent of the image (0-100). */
    public double getTA() {
        return inputs.ta;
    }

    /** Best target's lateral offset in camera space, meters (targetpose_cameraspace x). */
    public double getTargetSpaceX() {
        return inputs.targetSpaceX_m;
    }

    /** Best target's forward distance in camera space, meters (targetpose_cameraspace z). */
    public double getTargetSpaceZ() {
        return inputs.targetSpaceZ_m;
    }

    public int getPrimaryTagId() {
        return inputs.primaryTagId;
    }

    public int[] getVisibleTagIds() {
        return inputs.visibleTagIds;
    }

    // ---- Pose estimation ----
    public PoseObservation[] getPoseObservations() {
        return inputs.poseObservations;
    }

    // ---- Commands ----
    public void setPipeline(int index) {
        io.setPipeline(index);
    }

    public void setFiducialFilters(int[] ids) {
        io.setFiducialFilters(ids);
    }

    public void setRobotOrientation(Rotation2d heading) {
        io.setRobotOrientation(heading);
    }

    public void updateSimPose(Pose2d robotPose) {
        io.updateSimPose(robotPose);
    }
}
