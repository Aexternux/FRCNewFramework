package frc.robot.devices.swervemodule;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIO.SwerveModuleIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DriveConnected", driveConnected);
    table.put("DrivePos_r", drivePos_r);
    table.put("DriveVel_rps", driveVel_rps);
    table.put("DriveVolts_V", driveVolts_V);
    table.put("DriveCurrent_A", driveCurrent_A);
    table.put("SteerConnected", steerConnected);
    table.put("SteerAbsConnected", steerAbsConnected);
    table.put("SteerAbsPos_Rot2d", steerAbsPos_Rot2d);
    table.put("SteerPos_Rot2d", steerPos_Rot2d);
    table.put("SteerVel_rps", steerVel_rps);
    table.put("SteerVolts_V", steerVolts_V);
    table.put("SteerCurrent_A", steerCurrent_A);
    table.put("OdometryTimestamps_s", odometryTimestamps_s);
    table.put("OdometryDrivePos_r", odometryDrivePos_r);
    table.put("OdometrySteerPos_Rot2d", odometrySteerPos_Rot2d);
  }

  @Override
  public void fromLog(LogTable table) {
    driveConnected = table.get("DriveConnected", driveConnected);
    drivePos_r = table.get("DrivePos_r", drivePos_r);
    driveVel_rps = table.get("DriveVel_rps", driveVel_rps);
    driveVolts_V = table.get("DriveVolts_V", driveVolts_V);
    driveCurrent_A = table.get("DriveCurrent_A", driveCurrent_A);
    steerConnected = table.get("SteerConnected", steerConnected);
    steerAbsConnected = table.get("SteerAbsConnected", steerAbsConnected);
    steerAbsPos_Rot2d = table.get("SteerAbsPos_Rot2d", steerAbsPos_Rot2d);
    steerPos_Rot2d = table.get("SteerPos_Rot2d", steerPos_Rot2d);
    steerVel_rps = table.get("SteerVel_rps", steerVel_rps);
    steerVolts_V = table.get("SteerVolts_V", steerVolts_V);
    steerCurrent_A = table.get("SteerCurrent_A", steerCurrent_A);
    odometryTimestamps_s = table.get("OdometryTimestamps_s", odometryTimestamps_s);
    odometryDrivePos_r = table.get("OdometryDrivePos_r", odometryDrivePos_r);
    odometrySteerPos_Rot2d = table.get("OdometrySteerPos_Rot2d", odometrySteerPos_Rot2d);
  }

  public SwerveModuleIOInputsAutoLogged clone() {
    SwerveModuleIOInputsAutoLogged copy = new SwerveModuleIOInputsAutoLogged();
    copy.driveConnected = this.driveConnected;
    copy.drivePos_r = this.drivePos_r;
    copy.driveVel_rps = this.driveVel_rps;
    copy.driveVolts_V = this.driveVolts_V;
    copy.driveCurrent_A = this.driveCurrent_A;
    copy.steerConnected = this.steerConnected;
    copy.steerAbsConnected = this.steerAbsConnected;
    copy.steerAbsPos_Rot2d = this.steerAbsPos_Rot2d;
    copy.steerPos_Rot2d = this.steerPos_Rot2d;
    copy.steerVel_rps = this.steerVel_rps;
    copy.steerVolts_V = this.steerVolts_V;
    copy.steerCurrent_A = this.steerCurrent_A;
    copy.odometryTimestamps_s = this.odometryTimestamps_s.clone();
    copy.odometryDrivePos_r = this.odometryDrivePos_r.clone();
    copy.odometrySteerPos_Rot2d = this.odometrySteerPos_Rot2d.clone();
    return copy;
  }
}
