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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import java.util.Queue;

/** Phoenix 6 Pigeon2 implementation of {@link GyroIO}. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> yaw;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity;

  public GyroIOPigeon2() {
    pigeon = new Pigeon2(
        TunerConstants.DrivetrainConstants.Pigeon2Id,
        TunerConstants.DrivetrainConstants.CANBusName);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY_Hz);
    BaseStatusSignal.setUpdateFrequencyForAll(Constants.globalDelta_Hz, pitch, roll, yawVelocity);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, pitch, roll, yawVelocity).equals(StatusCode.OK);
    inputs.yaw_Rot2d = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.pitch_Rot2d = Rotation2d.fromDegrees(pitch.getValueAsDouble());
    inputs.roll_Rot2d = Rotation2d.fromDegrees(roll.getValueAsDouble());
    inputs.yawVel_radps = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.odometryYawPositions = yawPositionQueue.stream()
        .map(Rotation2d::fromDegrees)
        .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void zero(double angle) {
    pigeon.setYaw(angle);
  }
}
