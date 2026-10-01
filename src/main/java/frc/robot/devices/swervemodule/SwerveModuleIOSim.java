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

package frc.robot.devices.swervemodule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of swerve module IO. The sim models are configured using a set of
 * module constants from Phoenix. Simulation is always based on voltage control.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  // TunerConstants doesn't support separate sim constants, so they are declared locally.
  //
  // These mirror bot-2026's sim plant (VanguardConfig.DRIVE_SIM / STEER_SIM + MotorSim), and are
  // deliberately NOT the real hardware -- the steer is modelled ungeared and near-massless, so it
  // reaches its commanded angle almost instantly.
  //
  // This has to stay consistent with Drive's antiskew kSkew, which pre-rotates the velocity
  // command by (omega * kSkew) radians to cancel azimuth lag. kSkew is effectively "how many
  // seconds of steer lag am I cancelling". A heavier/geared steer plant here has ~44ms of lag
  // while kSkew (-0.002) only cancels ~2ms; the uncancelled remainder shows up as the robot
  // slanting sideways whenever it translates and rotates at once. A near-zero-lag plant leaves
  // nothing to cancel, so it tracks straight.
  //
  // Consequence: sim is a good parity check against bot-2026 and a poor predictor of on-field
  // skew, where the real geared steer does have real lag. Don't tune kSkew for the robot in here.
  private static final double DRIVE_KP = 0.05;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT =
      0.91035; // Same units as TunerConstants: (volt * secs) / rotation
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);

  /** bot-2026 STEER_SIM kP is 8.5 V per *rotation*; this controller works in radians. */
  private static final double TURN_KP = 8.5 / (2.0 * Math.PI);
  private static final double TURN_KD = 0.0;

  /** bot-2026 SimConfig inertia for both swerve motors. */
  private static final double SIM_INERTIA = 0.0001;

  /**
   * bot-2026's MotorSim builds its DCMotorSim from {@code Feedback.SensorToMechanismRatio} only.
   * Its steer motor uses a FusedCANcoder and sets just RotorToSensorRatio, leaving
   * SensorToMechanismRatio at the 1.0 default -- so the steer plant is ungeared in sim.
   */
  private static final double TURN_SIM_GEAR_RATIO = 1.0;

  private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
  private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public SwerveModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    // Create drive and turn sim models (bot-2026 sim plant -- see note on SIM_INERTIA above)
    driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DRIVE_GEARBOX, SIM_INERTIA, constants.DriveMotorGearRatio),
            DRIVE_GEARBOX);
    turnSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TURN_GEARBOX, SIM_INERTIA, TURN_SIM_GEAR_RATIO),
            TURN_GEARBOX);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(0.02);
    turnSim.update(0.02);

    // Update drive inputs
    inputs.driveConnected = true;
    inputs.drivePos_r = Units.radiansToRotations(driveSim.getAngularPositionRad());
    inputs.driveVel_rps = Units.radiansToRotations(driveSim.getAngularVelocityRadPerSec());
    inputs.driveVolts_V = driveAppliedVolts;
    inputs.driveCurrent_A = Math.abs(driveSim.getCurrentDrawAmps());

    // Update turn inputs
    inputs.steerConnected = true;
    inputs.steerAbsConnected = true;
    inputs.steerAbsPos_Rot2d = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.steerPos_Rot2d = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.steerVel_rps = Units.radiansToRotations(turnSim.getAngularVelocityRadPerSec());
    inputs.steerVolts_V = turnAppliedVolts;
    inputs.steerCurrent_A = Math.abs(turnSim.getCurrentDrawAmps());

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps_s = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePos_r = new double[] {inputs.drivePos_r};
    inputs.odometrySteerPos_Rot2d = new Rotation2d[] {inputs.steerPos_Rot2d};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }
}
