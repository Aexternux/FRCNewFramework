package frc.robot.devices.limelight;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import frc.robot.devices.limelight.LimelightIO.LimelightIOInputs;

public class LimelightIOInputsAutoLogged extends LimelightIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("HasTarget", hasTarget);
    table.put("Tx", tx);
    table.put("Ty", ty);
    table.put("Ta", ta);
    table.put("PrimaryTagId", primaryTagId);
    table.put("VisibleTagIds", visibleTagIds);
    table.put("PoseObservations", poseObservations);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    hasTarget = table.get("HasTarget", hasTarget);
    tx = table.get("Tx", tx);
    ty = table.get("Ty", ty);
    ta = table.get("Ta", ta);
    primaryTagId = table.get("PrimaryTagId", primaryTagId);
    visibleTagIds = table.get("VisibleTagIds", visibleTagIds);
    poseObservations = table.get("PoseObservations", poseObservations);
  }

  @Override
  public LimelightIOInputsAutoLogged clone() {
    LimelightIOInputsAutoLogged copy = new LimelightIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.hasTarget = this.hasTarget;
    copy.tx = this.tx;
    copy.ty = this.ty;
    copy.ta = this.ta;
    copy.primaryTagId = this.primaryTagId;
    copy.visibleTagIds = this.visibleTagIds.clone();
    copy.poseObservations = this.poseObservations.clone();
    return copy;
  }
}
