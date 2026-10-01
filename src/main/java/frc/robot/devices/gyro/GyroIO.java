// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.devices.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Hardware-independent input/output contract for a robot gyro. */
public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yaw_Rot2d = new Rotation2d();
    public Rotation2d pitch_Rot2d = new Rotation2d();
    public Rotation2d roll_Rot2d = new Rotation2d();
    public double yawVel_radps = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  /** Sets the reported yaw angle in degrees. */
  public default void zero(double angle) {}
}
