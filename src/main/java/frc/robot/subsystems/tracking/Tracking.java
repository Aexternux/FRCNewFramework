package frc.robot.subsystems.tracking;

import frc.robot.devices.limelight.Limelight;
import frc.robot.devices.limelight.LimelightConfig;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.Util;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

// Tracking = aim/drive the robot at an AprilTag. Same subsystem pattern as everything else:
// SubsystemBase<TrackingStates> (its state enum), private singleton, inputPeriodic/handle/outputPeriodic.
public class Tracking extends SubsystemBase<TrackingStates> {

  public static final int PIPELINE = 0;
  private static final int NO_TAG_ID = -1;
  private static final double CAMERA_X_m = 0.2;
  private static final double CAMERA_Y_m = 0.0;
  private static final double CAMERA_Z_m = 0.3;
  private static final double CAMERA_ROLL_deg = 0.0;
  private static final double CAMERA_PITCH_deg = -15.0;
  private static final double CAMERA_YAW_deg = 0.0;
  private static final double ALIGNMENT_TOLERANCE_m = 0.02;
  private static final double HALF_TURN_deg = 180.0;

  // Heading controller gains/limits (rotate the robot to a target angle).
  private static final double ROT_KP = 4.0;
  private static final double ROT_DEADBAND_deg = 1.0;
  private static final double MAX_ROT_radps = 4.0;
  private static final double TRANSLATE_ALIGN_TOL_deg = 10.0;
  private static final double STANDOFF_m = 1.0;

  private static Tracking instance;

  // The camera we track with, plus a handle to Drive (we need the robot's pose and to set overrides).
  private final Limelight limelight;
  private final Drive drive;
  // The field's tag positions, loaded once. Used to look up where a tag is on the field.
  private final AprilTagFieldLayout tagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  private boolean enabled = true;

  // targetTagId = the tag we're going for. locked = we've actually seen it, so stop re-choosing.
  private int targetTagId = NO_TAG_ID;
  private boolean locked = false;

  private double currTxTarget = 0.0;
  private double currTzTarget = 0.0;

  private Tracking() {
    super("Tracking");
    // Build the tracking camera: NT name "limelight-mvrt", mounted 0.2m fwd / 0.3m up / tilted up
    // 15deg, and (in sim) can only see STANDOFF... SIM_MAX_SIGHT_RANGE_m far.
    this.limelight = new Limelight(
        "Tracking/Limelight",
        new LimelightConfig("limelight-mvrt")
            .withRobotToCamera(CAMERA_X_m, CAMERA_Y_m, CAMERA_Z_m, CAMERA_ROLL_deg,
                CAMERA_PITCH_deg, CAMERA_YAW_deg)
            .withMaxSightRange(VisionConstants.SIM_MAX_SIGHT_RANGE_m));
    this.drive = Drive.getInstance();
    setCommand(TrackingStates.IDLE); // start in the idle state.
  }

  public static Tracking getInstance() {
    if (instance == null) {
      instance = new Tracking();
    }
    return instance;
  }


  // Lifecycle step 1: read the camera every loop, then decide which tag we're going for.
  @Override
  public void inputPeriodic() {
    limelight.updateSimPose(drive.getPose()); // sim only: tell the fake camera where the robot is.
    limelight.readInputs();                    // pull the latest detections off the camera.
    updateTarget();                            // pick / keep the tag to track.
  }

  // Decide which tag to track. Until we've actually seen a tag, keep picking the closest one to the
  // robot (by field position). Once we see it, "lock" so the choice stops changing mid-track. The
  // lock clears when the driver lets go of the tracking override.
  private void updateTarget() {
    if (!drive.isTrackingRequested()) {
      locked = false; // not tracking right now -> next press starts fresh.
    }
    if (locked) {
      return; // already committed to a tag; don't second-guess it.
    }
    targetTagId = nearestTagId(drive.getPose());
    if (targetTagId != NO_TAG_ID && contains(limelight.getVisibleTagIds(), targetTagId)) {
      locked = true; // the closest tag is in view -> commit to it.
    }
  }

  // Closest tag to the robot by straight-line field distance (uses the field layout, not the camera).
  private int nearestTagId(Pose2d robot) {
    int best = NO_TAG_ID;
    double bestDist_m = Double.POSITIVE_INFINITY;
    for (var tag : tagLayout.getTags()) {
      double dist_m = tag.pose.toPose2d().getTranslation().getDistance(robot.getTranslation());
      if (dist_m < bestDist_m) {
        bestDist_m = dist_m;
        best = tag.ID;
      }
    }
    return best;
  }

  // Where to drive to before we can see the tag: a point STANDOFF_m in front of the tag, facing it.
  // Drive uses this as the "drive to pose" goal until the camera picks the tag up.
  public Pose2d getApproachPose() {
    var tagPose = tagLayout.getTagPose(targetTagId);
    OptionalDouble angle = TagAngles.getTargetAngle(targetTagId); // the heading to face this tag.
    if (targetTagId == NO_TAG_ID || tagPose.isEmpty() || angle.isEmpty()) {
      return drive.getPose(); // nothing chosen -> "go where you already are" (a no-op).
    }
    Rotation2d heading = Rotation2d.fromDegrees(angle.getAsDouble());
    // Back off from the tag along that heading by STANDOFF_m.
    Translation2d approach = tagPose.get().toPose2d().getTranslation()
        .plus(new Translation2d(-STANDOFF_m, heading));
    return new Pose2d(approach, heading);
  }

  // Little helper: is `value` in the int array `arr`?
  private static boolean contains(int[] arr, int value) {
    for (int a : arr) {
      if (a == value) {
        return true;
      }
    }
    return false;
  }

  // Lifecycle step 2: the state machine. getCommand() is the current state; setCommand(...) changes
  // it. firstLoop() is true only on the first loop after entering a state (good for one-time setup).
  @Override
  protected void handle() {
    switch (getCommand()) {

      case DISABLED:
        if (enabled) setCommand(TrackingStates.IDLE);
        break;

      case IDLE:
        if (!enabled) setCommand(TrackingStates.DISABLED);
        else if (drive.isTrackingRequested()) setCommand(TrackingStates.TRACKING);
        break;

      case TRACKING:
        if (!enabled) {
          setCommand(TrackingStates.DISABLED);
        } else if (!drive.isTrackingRequested()) {
          setCommand(TrackingStates.IDLE);
        }
        break;

      default:
        break;
    }
  }


  // Lifecycle step 3: publish everything to the log so you can see it in AdvantageScope.
  @Override
  public void outputPeriodic() {
    Logger.recordOutput("Tracking/Enabled", enabled);
    Logger.recordOutput("Tracking/State", getCommand().toString());
    Logger.recordOutput("Tracking/TxTarget", currTxTarget);
    Logger.recordOutput("Tracking/TzTarget", currTzTarget);

    int tagId = getTargetTag();
    OptionalDouble targetAngle = TagAngles.getTargetAngle(tagId);
    Logger.recordOutput("Tracking/HasTarget", hasTarget());
    Logger.recordOutput("Tracking/TargetTagId", tagId);          // the tag we chose.
    Logger.recordOutput("Tracking/PrimaryTagId", getPrimaryTagId()); // what the camera thinks is best.
    Logger.recordOutput("Tracking/TargetAngle_deg", targetAngle.orElse(Double.NaN));

    // The chosen tag's field pose, as a 0- or 1-length array (drop on the 3D field to see it).
    Pose3d[] targetTagPose = tagLayout.getTagPose(tagId)
        .map(pose -> new Pose3d[] {pose})
        .orElse(new Pose3d[0]);
    Logger.recordOutput("Tracking/TargetTagPose", targetTagPose);

    // Draw a line from the camera to every tag it currently sees (camera -> tag, camera -> tag, ...).
    Pose3d cameraPose = new Pose3d(drive.getPose()).transformBy(limelight.getRobotToCamera());
    List<Pose3d> sightline = new ArrayList<>();
    for (int id : limelight.getVisibleTagIds()) {
      tagLayout.getTagPose(id).ifPresent(pose -> {
        sightline.add(cameraPose);
        sightline.add(pose);
      });
    }
    Logger.recordOutput("Tracking/Sightline", sightline.toArray(new Pose3d[0]));
  }

  // ============================================================================================
  // Older getTrackingSpeeds(...) overloads below. These were the earlier hand-tuned versions; the
  // current drive-to-pose + track flow uses getTargetSpeeds() / getApproachPose() further down and
  // does NOT call these. Left for reference.
  // ============================================================================================

  public ChassisSpeeds getTrackingSpeeds(double currAngle, double maxVel) {
    double rotSpeed = 0;
    double sideSpeed = 0;
    double forwardSpeed = 0;
    
    double txTol = 0;
    double tzTol = 0;

    double fMultiplier = -0.5; 
    double sMultiplier = 0.5; 
    double error = 0;
    double targetAngle = 180; //Could set to a tid switch case, seeing which ID and which angle corresponds
    
    if (currAngle < 0) { // This means that its closer to the left
      error = -(targetAngle - Math.abs(currAngle));
    } else { // this means that its closer to the right
      error = (targetAngle - Math.abs(currAngle));
    }

    rotSpeed = Math.abs(error) > 0.5 ? (int)(1.5 * error) : 0;

    if (error < 30 && hasTarget()) {
        TrackingLocation current = getLocation();
        double tx_error = txTol - (current.getTx());
        System.out.println(current.getTz());
        double tz_error = tzTol - (current.getTz()-1);
        
        double calculatedSpeedSide = 0;
        double calculatedSpeedForward = 0;
        double maxSpeed = maxVel;

        calculatedSpeedForward = SharkFinConfigs.BASE_SHARKFIN.calculate(tz_error, tx_error);
        calculatedSpeedSide = SharkFinConfigs.BASE_SHARKFIN.calculate(tx_error, tz_error);

        // clamp
        if (Math.abs(tz_error) > 0.02 || Math.abs(tx_error) > 0.02) {
            sideSpeed = Util.limit(calculatedSpeedSide, maxSpeed);
            forwardSpeed = Util.limit(calculatedSpeedForward, maxSpeed);
        }
    }
    // Cosine compensation
    double comp = Math.cos(Units.degreesToRadians(error));
    // System.out.println(fMultiplier * forwardSpeed * comp);
    return new ChassisSpeeds(fMultiplier * forwardSpeed * comp, sMultiplier * sideSpeed * comp, rotSpeed);
}

    public ChassisSpeeds getTrackingSpeeds(double targetAngle, double currAngle, double maxVel, double tZ_offset, double tX_offset) {
        double rotSpeed = 0;
        double sideSpeed = 0;
        double forwardSpeed = 0;
        
        if (!hasTarget()) {
            return new ChassisSpeeds();
        }
        double txTol = 0;
        double tzTol = 0;

        // tz_error/tx_error are negated distances (0 - getTz), so a tag ahead gives a negative
        // error; these multipliers flip that back so the robot drives TOWARD the tag, not away.
        double fMultiplier = -1;
        double sMultiplier = 1;

        // Shortest-path heading error, so it never chases the long way around ±180.
        double error = MathUtil.inputModulus(targetAngle - currAngle, -180.0, 180.0);
        rotSpeed = Math.abs(error) > ROT_DEADBAND_deg
                ? MathUtil.clamp(ROT_KP * Units.degreesToRadians(error), -MAX_ROT_radps, MAX_ROT_radps)
                : 0;

        if (Math.abs(error) < TRANSLATE_ALIGN_TOL_deg) {
            TrackingLocation current = getLocation();
            double tx_error = txTol - (current.getTx() - tZ_offset);
            double tz_error = tzTol - (current.getTz() - tX_offset);

            double calculatedSpeedSide = 0;
            double calculatedSpeedForward = 0;
            double maxSpeed = maxVel;

            calculatedSpeedForward = SharkFinConfigs.BASE_SHARKFIN.calculate(tz_error, tx_error);
            calculatedSpeedSide = SharkFinConfigs.BASE_SHARKFIN.calculate(tx_error, tz_error);

            // clamp
            if (Math.abs(tz_error) > 0.02 || Math.abs(tx_error) > 0.02) {
                sideSpeed = Util.limit(calculatedSpeedSide, maxSpeed);
                forwardSpeed = Util.limit(calculatedSpeedForward, maxSpeed);
            }
        }
        // Cosine compensation
        double comp = Math.cos(Units.degreesToRadians(error));
        return new ChassisSpeeds(fMultiplier * forwardSpeed * comp, sMultiplier * sideSpeed * comp, rotSpeed);
    }

    public ChassisSpeeds getTrackingSpeeds(double targetAngle, double currAngle, double maxVel) {
        double rotSpeed = 0;
        double sideSpeed = 0;
        double forwardSpeed = 0;
        
        if (!hasTarget()) {
            return new ChassisSpeeds();
        }
        double txTol = 0;
        double tzTol = 0;

        double fMultiplier = 1; 
        double sMultiplier = -1; 

        double error = targetAngle - currAngle;
        rotSpeed = Math.abs(error) > 0.5 ? ROT_KP * error : 0;

        if (error < 10) {
            TrackingLocation current = getLocation();
            double tx_error = txTol - (current.getTx());
            double tz_error = tzTol - (current.getTz());
            
            double calculatedSpeedSide = 0;
            double calculatedSpeedForward = 0;
            double maxSpeed = maxVel;
    
            calculatedSpeedForward = SharkFinConfigs.BASE_SHARKFIN.calculate(tz_error, tx_error);
            calculatedSpeedSide = SharkFinConfigs.BASE_SHARKFIN.calculate(tx_error, tz_error);

            // clamp
            if (Math.abs(tz_error) > 0.02 || Math.abs(tx_error) > 0.02) {
                sideSpeed = Util.limit(calculatedSpeedSide, maxSpeed);
                forwardSpeed = Util.limit(calculatedSpeedForward, maxSpeed);
            }
        }
        // Cosine compensation
        double comp = Math.cos(Units.degreesToRadians(error));
        System.out.println(fMultiplier * forwardSpeed * comp);

        return new ChassisSpeeds(fMultiplier * forwardSpeed * comp, sMultiplier * sideSpeed * comp, rotSpeed);
    }


    public ChassisSpeeds getTrackingSpeeds(double currAngle) {
      double rotSpeed = 0;
      double targetAngle = 0;
      double error = 0;
    
      double targetStartAngleRight = -5;
      double targetStartAngleLeft = 5;

      if (hasTarget()) {
        error = targetAngle - getTx();
        rotSpeed = Math.abs(error) > 0.5 ?(int) (ROT_KP * error) : 0;
      }
      // } else {
      //   if (currAngle >= targetStartAngleLeft) { // This means that its closer to the left
      //     System.out.println(currAngle);
      //     rotSpeed = -(currAngle - targetStartAngleLeft) * ROT_KP;
      //   } else if (currAngle <= targetStartAngleRight) { // this means that its closer to the right
      //     rotSpeed = -(currAngle - targetStartAngleRight) * ROT_KP;
      //   }
      // }
      return new ChassisSpeeds(0,0,rotSpeed);
  }
  public boolean hasTarget() {
    return limelight.hasTarget();
  }

  /** The id of the primary in-view AprilTag, or -1 if none. */
  public int getPrimaryTagId() {
    return limelight.getPrimaryTagId();
  }

  public boolean isConnected() {
    return limelight.isConnected();
  }

  public double getTx() {
    return limelight.getTX();
  }

  public double getTa() {
    return limelight.getTA();
  }

  public double getTaLinear() {
    return Math.sqrt(getTa());
  }

  public double getTz() {
    return limelight.getTargetSpaceZ();
  }

  public double get3dTx() {
    return limelight.getTargetSpaceX();
  }

  public void setTxTarget(double tx) {
    currTxTarget = tx;
  }

  public void setTzTarget(double tz) {
    currTzTarget = tz;
  }

  public double getTxTarget() {
    return currTxTarget;
  }

  public double getTzTarget() {
    return currTzTarget;
  }

  public TrackingLocation getLocation() {
    return new TrackingLocation(get3dTx(), getTz());
  }

  // ---- Active tracking API (what Drive's tracking override actually calls) ----

  // The tag we're going for.
  public int getTargetTag() {
    return targetTagId;
  }

  // Is that chosen tag in the camera's view right now? Drive uses this to switch between "drive to
  // the approach pose" (false) and "run the final tracking" (true).
  public boolean seesTarget() {
    return targetTagId != NO_TAG_ID && contains(limelight.getVisibleTagIds(), targetTagId);
  }

  // Where the chosen tag is relative to the robot, computed from odometry + the field layout.
  // Returns forward distance and lateral (right-positive) offset, matching the camera's tx/tz.
  public TrackingLocation getTargetLocation() {
    var tagPose = tagLayout.getTagPose(targetTagId);
    if (tagPose.isEmpty()) {
      return new TrackingLocation(0.0, 0.0);
    }
    Pose2d robot = drive.getPose();
    // tag position relative to the robot: subtract the robot's position, then un-rotate by heading.
    Translation2d rel = tagPose.get().toPose2d().getTranslation()
        .minus(robot.getTranslation())
        .rotateBy(robot.getRotation().unaryMinus());
    return new TrackingLocation(-rel.getY(), rel.getX()); // (tx = right, tz = forward)
  }

  // The final tracking move once the tag is in view: turn to its angle AND drive in to STANDOFF_m.
  public ChassisSpeeds getTargetSpeeds(double currAngle, double maxVel) {
    OptionalDouble angle = TagAngles.getTargetAngle(targetTagId);
    if (targetTagId == NO_TAG_ID || angle.isEmpty()) {
      return new ChassisSpeeds();
    }

    double rotSpeed = rotationTo(currAngle, angle.getAsDouble()); // turn toward the tag's angle.

    double forwardSpeed = 0.0;
    double sideSpeed = 0.0;
    TrackingLocation current = getTargetLocation();
    double tx_error = -current.getTx();               // want lateral offset = 0.
    double tz_error = STANDOFF_m - current.getTz();   // want forward distance = STANDOFF_m.
    // Only drive if we're not basically there. SharkFin is the approach curve (smooth in, no overshoot).
    if (Math.abs(tz_error) > ALIGNMENT_TOLERANCE_m || Math.abs(tx_error) > ALIGNMENT_TOLERANCE_m) {
      forwardSpeed = Util.limit(SharkFinConfigs.BASE_SHARKFIN.calculate(tz_error, tx_error), maxVel);
      sideSpeed = Util.limit(SharkFinConfigs.BASE_SHARKFIN.calculate(tx_error, tz_error), maxVel);
    }

    return new ChassisSpeeds(-forwardSpeed, sideSpeed, rotSpeed);
  }

  // Just the turn-to-the-tag part (Drive uses this to steer while driving to the approach pose, so it
  // doesn't inherit the pose follower's overly aggressive rotation).
  public double approachRotation(double currAngle) {
    OptionalDouble angle = TagAngles.getTargetAngle(targetTagId);
    return angle.isEmpty() ? 0.0 : rotationTo(currAngle, angle.getAsDouble());
  }

  // Shared heading controller: P on the wrapped error, clamped, with a deadband. Degrees in, rad/s out.
  private double rotationTo(double currAngle, double targetAngle) {
    // inputModulus wraps the error to [-180, 180) so it always turns the short way.
    double error = MathUtil.inputModulus(targetAngle - currAngle, -HALF_TURN_deg, HALF_TURN_deg);
    return Math.abs(error) > ROT_DEADBAND_deg
        ? MathUtil.clamp(ROT_KP * Units.degreesToRadians(error), -MAX_ROT_radps, MAX_ROT_radps)
        : 0.0;
  }

  public void enable() {
    enabled = true;
    setCommand(TrackingStates.IDLE);
  }

  public void disable() {
    enabled = false;
    setCommand(TrackingStates.DISABLED);
  }

  public void toggleEnabled() {
    enabled = !enabled;
    setCommand(enabled ? TrackingStates.IDLE : TrackingStates.DISABLED);
  }

  public boolean getEnabled() {
    return enabled;
  }

  public boolean isEnabled() {
    return enabled;
  }

  public void setPipeline(int pipeline) {
    limelight.setPipeline(pipeline);
  }

  public void setValidIds(double[] validIds) {
    int[] ids = new int[validIds.length];
    for (int i = 0; i < validIds.length; i++) {
      ids[i] = (int) validIds[i];
    }
    limelight.setFiducialFilters(ids);
  }
}
