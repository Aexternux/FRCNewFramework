package frc.robot.devices.motor;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.List;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.Constants;
import frc.robot.devices.motor.MotorConfig.GravityType;

public class MotorIOTalonFX implements MotorIO {

    private final MotorConfig config;
    private final TalonFX talon;
    private final List<TalonFX> followers;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<AngularAcceleration> acceleration;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> tempC;

    private final VoltageOut voltageControl;
    private final DutyCycleOut dutyCycleControl;
    private final VelocityVoltage velocityControl;
    private final PositionVoltage positionControl;
    private final MotionMagicVoltage motionMagicControl;
    private final MotionMagicVelocityVoltage motionMagicVelocityControl;
    private final NeutralOut neutralControl = new NeutralOut();

    public MotorIOTalonFX(MotorConfig config) {
        this.config = config;
        this.talon = new TalonFX(config.canId, config.canbus);
        this.followers = config.followers.stream()
                .map(followerConfig -> new TalonFX(followerConfig.canId(), config.canbus))
                .toList();

        boolean foc = config.foc;
        voltageControl = new VoltageOut(0).withEnableFOC(foc);
        dutyCycleControl = new DutyCycleOut(0).withEnableFOC(foc);
        velocityControl = new VelocityVoltage(0).withEnableFOC(foc);
        positionControl = new PositionVoltage(0).withEnableFOC(foc);
        motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(foc);
        motionMagicVelocityControl = new MotionMagicVelocityVoltage(0).withEnableFOC(foc);

        applyConfig();

        for (int i = 0; i < followers.size(); i++) {
            followers.get(i).setControl(new Follower(
                    talon.getDeviceID(), config.followers.get(i).alignment()));
        }

        position = talon.getPosition();
        velocity = talon.getVelocity();
        acceleration = talon.getAcceleration();
        appliedVolts = talon.getMotorVoltage();
        supplyCurrent = talon.getSupplyCurrent();
        statorCurrent = talon.getStatorCurrent();
        tempC = talon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.globalDelta_Hz,
                position, velocity, acceleration, appliedVolts, supplyCurrent, statorCurrent, tempC);
        talon.optimizeBusUtilization();
    }

    private void applyConfig() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimit;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        if (config.statorCurrentLimit != null) {
            cfg.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit;
            cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        }
        cfg.Feedback.SensorToMechanismRatio = config.sensorToMechanismRatio;
        cfg.MotorOutput.Inverted =
                config.inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        cfg.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        double unitsPerRotation = config.unitsPerRotation();
        cfg.MotionMagic.MotionMagicCruiseVelocity = config.mmCruiseVelocity / unitsPerRotation;
        cfg.MotionMagic.MotionMagicAcceleration = config.mmAcceleration / unitsPerRotation;
        cfg.MotionMagic.MotionMagicJerk = config.mmJerk / unitsPerRotation;

        cfg.Slot0.GravityType = toGravityValue(config.gravity);
        cfg.Slot0.kS = config.kS;
        cfg.Slot0.kV = config.kV;
        cfg.Slot0.kA = config.kA;
        cfg.Slot0.kG = config.kG;
        cfg.Slot0.kP = config.kP;
        cfg.Slot0.kI = config.kI;
        cfg.Slot0.kD = config.kD;

        tryUntilOk(5, () -> talon.getConfigurator().apply(cfg, 0.25));
        for (TalonFX follower : followers) {
            tryUntilOk(5, () -> follower.getConfigurator().apply(cfg, 0.25));
        }
    }

    private static GravityTypeValue toGravityValue(GravityType type) {
        switch (type) {
            case ELEVATOR:
                return GravityTypeValue.Elevator_Static;
            case ARM:
                return GravityTypeValue.Arm_Cosine;
            case NONE:
            default:
                return GravityTypeValue.Elevator_Static;
        }
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        var status = BaseStatusSignal.refreshAll(
                position, velocity, acceleration, appliedVolts, supplyCurrent, statorCurrent, tempC);
        inputs.connected = status.isOK();
        inputs.position = position.getValueAsDouble();
        inputs.velocity = velocity.getValueAsDouble();
        inputs.acceleration = acceleration.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
        inputs.statorCurrent = statorCurrent.getValueAsDouble();
        inputs.tempC = tempC.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        talon.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void setOpenLoop(double dutyCycle) {
        talon.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void setVelocity(double velocity) {
        talon.setControl(velocityControl.withVelocity(velocity));
    }

    @Override
    public void setPositionVoltage(double position) {
        talon.setControl(positionControl.withPosition(position));
    }

    @Override
    public void setMotionMagic(double position) {
        talon.setControl(motionMagicControl.withPosition(position));
    }

    @Override
    public void setMotionMagicVelocity(double velocity) {
        talon.setControl(motionMagicVelocityControl.withVelocity(velocity));
    }

    @Override
    public void stop() {
        talon.setControl(neutralControl);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        talon.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void zeroPosition(double position) {
        talon.setPosition(position);
    }
}
