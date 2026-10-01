package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.generated.TunerConstants;

public final class ElevatorConstants {

    private ElevatorConstants() {}

    public static final int MOTOR_ID = 15;
    public static final int FOLLOWER_ID = 16;
    public static final MotorAlignmentValue FOLLOWER_OPPOSE = MotorAlignmentValue.Opposed;
    public static final String CANBUS = TunerConstants.kCANBus.getName();
    public static final int HALL_EFFECT_CHANNEL = 0;

    public static final boolean INVERTED = true;
    public static final boolean BRAKE = true;
    public static final double SUPPLY_CURRENT_LIMIT_A = 40.0;

    public static final double kS = 0.07;
    public static final double kV = 0.75;
    public static final double kA = 0.0;
    public static final double kG = 1.0;
    public static final double kP = 18.4;
    public static final double kI = 0.0;
    public static final double kD = 1.5;
    public static final GravityType GRAVITY = GravityType.ELEVATOR;

    public static final double MM_CRUISE_VELOCITY = 3.1;
    public static final double MM_ACCELERATION = 23.5;
    public static final double MM_JERK = 150.0;

    public static final DCMotor SIM_MOTOR = DCMotor.getKrakenX60Foc(2);
    public static final double SIM_MOI = 0.1;

    public static final boolean HALL_INVERTED = true;
    public static final double HALL_DEBOUNCE_s = 0.1;
    public static final DebounceType HALL_DEBOUNCE_TYPE = DebounceType.kRising;

    public static final double MIN_HEIGHT_m = 0.2191;
    public static final double MAX_HEIGHT_m = 1.724;
    public static final double STROKE_m = MAX_HEIGHT_m - MIN_HEIGHT_m;
    public static final double TOLERANCE_m = 0.5;

    public static final double MIN_HEIGHT_R = -14.812;
    public static final double MAX_HEIGHT_R = 1.262;
    public static final double METERS_TO_ROTATIONS = (MAX_HEIGHT_R - MIN_HEIGHT_R) / STROKE_m;

    public static final double HOMING_VOLTS = -0.5;
}
