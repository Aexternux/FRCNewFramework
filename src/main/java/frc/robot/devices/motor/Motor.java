package frc.robot.devices.motor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

public class Motor {

    public enum Mode {
        REAL,
        SIM
    }

    public enum ControlType {
        VOLTAGE,
        OPEN_LOOP,
        VELOCITY,
        POSITION_VOLTAGE,
        MOTION_MAGIC,
        MOTION_MAGIC_VELOCITY,
        STOP
    }

    private final String name;
    private final String subsystemName;

    private final MotorIO io;
    private final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
    private final MotorConfig.MeasurementUnit measurementUnit;

    private ControlType control = ControlType.STOP;
    private double target = 0.0;

    public Motor(String name, String subsystemName, MotorConfig config, Mode mode) {
        this.name = name;
        this.subsystemName = subsystemName;
        this.measurementUnit = config.measurementUnit;

        if (mode == Mode.REAL) {
            this.io = new MotorIOTalonFX(config);
        } else if (Constants.physicsSim) {
            this.io = new MotorIOPhysicsSim(subsystemName + "/" + name, config);
        } else {
            this.io = new MotorIOSim(config);
        }
    }

    public Motor(String name, String subsystemName, MotorConfig config) {
        this(name, subsystemName, config, Constants.currentMode == Constants.Mode.REAL ? Mode.REAL : Mode.SIM);
    }

    public void readInputs() {
        io.updateInputs(inputs);
        inputs.position = measurementUnit.fromMechanismRotations(inputs.position);
        inputs.velocity = measurementUnit.fromMechanismRotations(inputs.velocity);
        inputs.acceleration = measurementUnit.fromMechanismRotations(inputs.acceleration);
        Logger.processInputs(subsystemName + "/" + name, inputs);
        Logger.recordOutput(subsystemName + "/" + name + "/MeasurementUnit", measurementUnit.name());

        if (measurementUnit != MotorConfig.MeasurementUnit.NONE) {
            Logger.recordOutput(subsystemName + "/" + name + "/Position" + measurementUnit.positionSuffix, inputs.position);
            Logger.recordOutput(subsystemName + "/" + name + "/Velocity" + measurementUnit.velocitySuffix, inputs.velocity);
            Logger.recordOutput(subsystemName + "/" + name + "/Acceleration" + measurementUnit.accelerationSuffix, inputs.acceleration);
        }
    }

    private void setOutput(ControlType control, double target) {
        this.control = control;
        this.target = target;
        Logger.recordOutput(subsystemName + "/" + name + "/ControlType", control);
        Logger.recordOutput(subsystemName + "/" + name + "/TargetSetpoint", target);
    }

    public void setVoltage(double volts) {
        // System.out.println("Volts: " + volts);
        volts = MathUtil.clamp(volts, -12, 12);
        setOutput(ControlType.VOLTAGE, volts);
        io.setVoltage(volts);
    }

    public void setOpenLoop(double dutyCycle) {
        setOutput(ControlType.OPEN_LOOP, dutyCycle);
        io.setOpenLoop(dutyCycle);
    }

    public void setVelocity(double velocity) {
        setOutput(ControlType.VELOCITY, velocity);
        io.setVelocity(measurementUnit.toMechanismRotations(velocity));
    }

    public void setPositionVoltage(double position) {
        setOutput(ControlType.POSITION_VOLTAGE, position);
        io.setPositionVoltage(measurementUnit.toMechanismRotations(position));
    }

    public void setMotionMagic(double position) {
        setOutput(ControlType.MOTION_MAGIC, position);
        io.setMotionMagic(measurementUnit.toMechanismRotations(position));
    }

    public void setMotionMagicVelocity(double velocity) {
        setOutput(ControlType.MOTION_MAGIC_VELOCITY, velocity);
        io.setMotionMagicVelocity(measurementUnit.toMechanismRotations(velocity));
    }

    /** Motion Magic velocity with a per-request acceleration; 0 uses the configured value. */
    public void setMotionMagicVelocity(double velocity, double acceleration) {
        setOutput(ControlType.MOTION_MAGIC_VELOCITY, velocity);
        io.setMotionMagicVelocity(measurementUnit.toMechanismRotations(velocity),
                measurementUnit.toMechanismRotations(acceleration));
    }

    /** Motion Magic velocity with an extra voltage feedforward added to the closed-loop output. */
    public void setMotionMagicVelocity(double velocity, double acceleration,
            double feedForwardVolts) {
        setOutput(ControlType.MOTION_MAGIC_VELOCITY, velocity);
        io.setMotionMagicVelocity(measurementUnit.toMechanismRotations(velocity),
                measurementUnit.toMechanismRotations(acceleration), feedForwardVolts);
    }

    public void stop() {
        setOutput(ControlType.STOP, 0.0);
        io.stop();
    }

    public void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }

    public void zeroPosition(double position) {
        io.zeroPosition(measurementUnit.toMechanismRotations(position));
    }

    public double getPosition() {
        return inputs.position;
    }

    public double getVelocity() {
        return inputs.velocity;
    }

    public double getAcceleration() {
        return inputs.acceleration;
    }

    public double getVoltage() {
        return inputs.appliedVolts;
    }

    public double getCurrent() {
        return inputs.statorCurrent;
    }

    /** True when stator current meets the spike threshold (e.g. hardstop stall during homing). */
    public boolean getCurrentSpike(double threshold_A) {
        return inputs.statorCurrent >= threshold_A;
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public MotorConfig.MeasurementUnit getMeasurementUnit() {
        return measurementUnit;
    }

    public ControlType getControl() {
        return control;
    }

    public double getSetpoint() {
        return target;
    }
}
