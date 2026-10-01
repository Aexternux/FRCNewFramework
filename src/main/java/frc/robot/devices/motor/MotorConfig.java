package frc.robot.devices.motor;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.system.plant.DCMotor;


public class MotorConfig {

    public enum GravityType {
        NONE,
        ELEVATOR,
        ARM
    }

    public enum MeasurementUnit {
        METERS("_m", "_mps", "_mps2"),
        DEGREES("_deg", "_degps", "_degps2"),
        ROTATIONS("_rot", "_rotps", "_rotps2"),
        RADIANS("_rad", "_radps", "_radps2"),
        NONE("", "", "");

        public final String positionSuffix;
        public final String velocitySuffix;
        public final String accelerationSuffix;

        MeasurementUnit(String positionSuffix, String velocitySuffix, String accelerationSuffix) {
            this.positionSuffix = positionSuffix;
            this.velocitySuffix = velocitySuffix;
            this.accelerationSuffix = accelerationSuffix;
        }

        /** Number of configured units in one mechanism rotation. */
        public double unitsPerRotation() {
            return switch (this) {
                case DEGREES -> 360.0;
                case RADIANS -> 2.0 * Math.PI;
                // Linear mechanism units are calibrated by sensorToMechanismRatio.
                case METERS, ROTATIONS, NONE -> 1.0;
            };
        }

        public double fromMechanismRotations(double rotations) {
            return rotations * unitsPerRotation();
        }

        public double toMechanismRotations(double value) {
            return value / unitsPerRotation();
        }
    }

    public int canId;
    public String canbus = "rio";
    /** Motors that should follow this config's leader TalonFX in hardware. */
    public final List<FollowerConfig> followers = new ArrayList<>();

    public boolean inverted = false;
    public boolean brake = true;
    public double supplyCurrentLimit = 40.0;
    /** Null leaves the TalonFX stator-current limit disabled. */
    public Double statorCurrentLimit = null;
    public boolean foc = true;

    public double sensorToMechanismRatio = 1.0;

    public double kS = 0.0;
    public double kV = 0.0;
    public double kA = 0.0;
    public double kG = 0.0;
    public double kP = 0.0;
    public double kI = 0.0;
    public double kD = 0.0;
    public GravityType gravity = GravityType.NONE;

    public double mmCruiseVelocity = 0.0;
    public double mmAcceleration = 0.0;
    public double mmJerk = 0.0;

    public MeasurementUnit measurementUnit = MeasurementUnit.NONE;

    public DCMotor simMotor = DCMotor.getKrakenX60Foc(1);
    public double simGearing = 1.0;
    public double simMOI = 0.001;

    public double unitsPerRotation() {
        return measurementUnit.unitsPerRotation();
    }

    public MotorConfig(int canId) {
        this.canId = canId;
    }

    public record FollowerConfig(int canId, MotorAlignmentValue alignment) {}

    public MotorConfig withCanbus(String canbus) {
        this.canbus = canbus;
        return this;
    }

    public MotorConfig withFollower(int followerId, MotorAlignmentValue oppose) {
        this.followers.add(new FollowerConfig(followerId, oppose));
        return this;
    }

    public MotorConfig withInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public MotorConfig withBrake(boolean brake) {
        this.brake = brake;
        return this;
    }

    public MotorConfig withSupplyCurrentLimit(double amps) {
        this.supplyCurrentLimit = amps;
        return this;
    }

    public MotorConfig withStatorCurrentLimit(double amps) {
        this.statorCurrentLimit = amps;
        return this;
    }

    public MotorConfig withFoc(boolean foc) {
        this.foc = foc;
        return this;
    }

    public MotorConfig withSensorToMechanismRatio(double ratio) {
        this.sensorToMechanismRatio = ratio;
        return this;
    }

    public MotorConfig withFFGains(
            double kS, double kV, double kA, double kG) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
        return this;
    }

    public MotorConfig withPIDGains( double kP, double kI, double kD, GravityType gravity) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.gravity = gravity;
        return this;
    }

    public MotorConfig withMotionMagic(double cruiseVelocity, double acceleration, double jerk) {
        this.mmCruiseVelocity = cruiseVelocity;
        this.mmAcceleration = acceleration;
        this.mmJerk = jerk;
        return this;
    }

    public MotorConfig withSim(DCMotor simMotor, double simGearing, double simMOI) {
        this.simMotor = simMotor;
        this.simGearing = simGearing;
        this.simMOI = simMOI;
        return this;
    }

    public MotorConfig withMeasurementUnit(MeasurementUnit measurementUnit) {
        this.measurementUnit = measurementUnit;
        return this;
    }
}
