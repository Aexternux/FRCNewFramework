package frc.robot.devices.swervemodule;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for a single swerve module (drive TalonFX + steer TalonFX + CANcoder).
 *
 * <p>Unlike the general-purpose {@code Motor} device, this device intentionally exposes the raw
 * drive/steer position samples collected by the {@code PhoenixOdometryThread} at the odometry
 * frequency, and drives the steer motor off a fused CANcoder with continuous wrap.
 */
public interface SwerveModuleIO {
  @AutoLog
  public static class SwerveModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePos_r = 0.0;
    public double driveVel_rps = 0.0;
    public double driveVolts_V = 0.0;
    public double driveCurrent_A = 0.0;

    public boolean steerConnected = false;
    public boolean steerAbsConnected = false;
    public Rotation2d steerAbsPos_Rot2d = new Rotation2d();
    public Rotation2d steerPos_Rot2d = new Rotation2d();
    public double steerVel_rps = 0.0;
    public double steerVolts_V = 0.0;
    public double steerCurrent_A = 0.0;

    public double[] odometryTimestamps_s = new double[] {};
    public double[] odometryDrivePos_r = new double[] {};
    public Rotation2d[] odometrySteerPos_Rot2d = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /** Run the drive motor at the specified open loop value. */
  public default void setDriveOpenLoop(double output) {}

  /** Run the turn motor at the specified open loop value. */
  public default void setTurnOpenLoop(double output) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocityRadPerSec) {}

  /** Run the turn motor to the specified rotation. */
  public default void setTurnPosition(Rotation2d rotation) {}

  public default void stop() {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setBrake(boolean brake) {}
}
