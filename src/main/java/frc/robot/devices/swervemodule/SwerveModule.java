// Copyright 2021-2024 FRC 6328
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.Constants;

import org.littletonrobotics.junction.Logger;

/**
 * A single swerve module device: drive TalonFX + steer TalonFX + fused CANcoder. Owns its own IO
 * (real / sim / replay) selected from {@link Constants#currentMode}, mirroring the rest of the
 * device system ({@code Motor}, {@code Limelight}, ...).
 */
public class SwerveModule {

    /** Drive control strategy chosen by {@code Drive} each cycle. */
    public enum Mode {
        HIGH_SPEED,
        HIGH_CONTROL
    }

    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final int index;

    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants;

    private double driveSetpoint_mps = 0;
    private Rotation2d steerSetpoint_Rot2d = null;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    public SwerveModule(int index,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        this.index = index;
        this.constants = constants;
        this.io = switch (Constants.currentMode) {
            case REAL -> new SwerveModuleIOTalonFX(constants);
            case SIM -> new SwerveModuleIOSim(constants);
            default -> new SwerveModuleIO() {}; // REPLAY: blank IO, inputs replayed from log
        };

        setBrakeMode(true);

        driveDisconnectedAlert = new Alert(
                "Disconnected drive motor on module " + Integer.toString(index) + ".",
                AlertType.kError);
        turnDisconnectedAlert = new Alert(
                "Disconnected steer motor on module " + Integer.toString(index) + ".", AlertType.kError);
        turnEncoderDisconnectedAlert = new Alert(
                "Disconnected steer encoder on module " + Integer.toString(index) + ".",
                AlertType.kError);
    }

    /**
     * Update inputs without running the rest of the periodic logic. This is useful
     * since these updates need to be
     * properly thread-locked.
     */
    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public void inputPeriodic() {
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps_s.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = Units.rotationsToRadians(inputs.odometryDrivePos_r[i]) * constants.WheelRadius;
            Rotation2d angle = inputs.odometrySteerPos_Rot2d[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.steerConnected);
        turnEncoderDisconnectedAlert.set(!inputs.steerAbsConnected);
    }

    public void outputPeriodic(Mode mode) {

        // Run closed loop turn control
        if (steerSetpoint_Rot2d != null) {
            io.setTurnPosition(steerSetpoint_Rot2d);

            // Drive: open-loop voltage (bot-2026 parity). volts = 12V * speed / maxSpeed.
            // driveSetpoint_mps is already optimized + cosine-scaled in runSetpoint().
            io.setDriveOpenLoop(12.0 * driveSetpoint_mps / constants.SpeedAt12Volts);
        }
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized
     * state.
     */
    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        state.optimize(getAngle());
        state.cosineScale(inputs.steerPos_Rot2d);

        // Update setpoints, controllers run in "periodic"
        steerSetpoint_Rot2d = state.angle;
        driveSetpoint_mps = state.speedMetersPerSecond;

        return state;
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setTurnPosition(new Rotation2d());
    }

    /** Disables all outputs to motors. */
    public void stop() {
        io.setTurnOpenLoop(0);
        io.setDriveOpenLoop(0);

        // Disable closed loop control for turn and drive
        steerSetpoint_Rot2d = null;
        driveSetpoint_mps = 0;
    }

    /** Sets whether brake mode is enabled. */
    public void setBrakeMode(boolean enabled) {
        io.setBrake(enabled);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.steerPos_Rot2d;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return Units.rotationsToRadians(inputs.drivePos_r) * constants.WheelRadius;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return Units.rotationsToRadians(inputs.driveVel_rps) * constants.WheelRadius;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps_s;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return Units.rotationsToRadians(inputs.drivePos_r);
    }

    /** Returns the module velocity in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        return inputs.driveVel_rps;
    }
}
