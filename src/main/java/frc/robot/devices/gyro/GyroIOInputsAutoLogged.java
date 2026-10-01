package frc.robot.devices.gyro;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Generated-style AdvantageKit input logger for {@link GyroIO.GyroIOInputs}. */
public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("Yaw_Rot2d", yaw_Rot2d);
    table.put("Pitch_Rot2d", pitch_Rot2d);
    table.put("Roll_Rot2d", roll_Rot2d);
    table.put("YawVel_radps", yawVel_radps);
    table.put("OdometryYawTimestamps", odometryYawTimestamps);
    table.put("OdometryYawPositions", odometryYawPositions);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    yaw_Rot2d = table.get("Yaw_Rot2d", yaw_Rot2d);
    pitch_Rot2d = table.get("Pitch_Rot2d", pitch_Rot2d);
    roll_Rot2d = table.get("Roll_Rot2d", roll_Rot2d);
    yawVel_radps = table.get("YawVel_radps", yawVel_radps);
    odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
    odometryYawPositions = table.get("OdometryYawPositions", odometryYawPositions);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.yaw_Rot2d = this.yaw_Rot2d;
    copy.pitch_Rot2d = this.pitch_Rot2d;
    copy.roll_Rot2d = this.roll_Rot2d;
    copy.yawVel_radps = this.yawVel_radps;
    copy.odometryYawTimestamps = this.odometryYawTimestamps.clone();
    copy.odometryYawPositions = this.odometryYawPositions.clone();
    return copy;
  }
}
