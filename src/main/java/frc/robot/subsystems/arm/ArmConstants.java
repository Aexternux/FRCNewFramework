package frc.robot.subsystems.arm;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.devices.motor.MotorConfig.GravityType;
import frc.robot.devices.motor.MotorConfig.MeasurementUnit;

public final class ArmConstants {
    private ArmConstants() {}

    public static final int MOTOR_ID = 0;
    public static final int FOLLOWER_ID = 1;
    public static final MotorAlignmentValue FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed;
    public static final String CANBUS = "rio";

    public static final boolean INVERTED = false;
    public static final boolean BRAKE = true;
    public static final double SUPPLY_CURRENT_LIMIT_A = 40.0;
    public static final boolean FOC_ENABLED = true;
    public static final double SIM_GEARING = 60.0;
    public static final double SENSOR_TO_MECHANISM_RATIO = SIM_GEARING;
    public static final MeasurementUnit MEASUREMENT_UNIT = MeasurementUnit.DEGREES;

    // Phoenix PID/feedforward gains use mechanism rotations and rotations per second.
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kG = 0.0;
    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 2.0;
    public static final GravityType GRAVITY_TYPE = GravityType.ARM;

    public static final double MM_CRUISE_VELOCITY_degps = 90.0;
    public static final double MM_ACCELERATION_degps2 = 180.0;
    public static final double MM_JERK_degps3 = 1000.0;

    public static final DCMotor SIM_MOTOR = DCMotor.getKrakenX60Foc(2);
    public static final double SIM_MOI_kgm2 = 0.08;

    public static final double ANGLE_TOLERANCE_deg = 0.5;
    public static final double IDLE_VOLTS = 0.0;
    public static final double STOW_ANGLE_deg = 0.0;
    public static final double RIGHT_TRIGGER_ANGLE_deg = 50.0;
    public static final double LEFT_TRIGGER_ANGLE_deg = 90.0;
    public static final double TRIGGER_THRESHOLD = 0.8;

    public static final double VISUAL_LENGTH_m = 1.0;
    public static final double VISUAL_CANVAS_WIDTH_m = 4.0;
    public static final double VISUAL_CANVAS_HEIGHT_m = 4.0;
    public static final double VISUAL_PIVOT_X_m = 2.0;
    public static final double VISUAL_PIVOT_Y_m = 2.0;
    public static final double VISUAL_LIGAMENT_THICKNESS_px = 10.0;
    public static final int MEASURED_COLOR_RED = 200;
    public static final int MEASURED_COLOR_GREEN = 0;
    public static final int MEASURED_COLOR_BLUE = 0;
    public static final int SETPOINT_COLOR_RED = 100;
    public static final int SETPOINT_COLOR_GREEN = 100;
    public static final int SETPOINT_COLOR_BLUE = 100;
}
