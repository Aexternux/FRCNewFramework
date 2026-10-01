package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.subsystems.SubsystemBase;

public class Arm extends SubsystemBase<Arm.Command>{
    
    public enum Command {
        DISABLED,
        IDLE,
        HOMING,
        GO_TO_ANGLE,
        MANUAL
    }

    public enum TravelSubstate {
        MOVING,
        HOLDING
    }

    private static Arm instance;
    private final Motor armDrivingMotor_2;
    private final MotorConfig armDrivingMotorConfig;

    private double targetAngle_deg = STOW_ANGLE_deg;
    private double targetVolts = IDLE_VOLTS;

    private final Arm2d measured2d = new Arm2d("Arm/Measured2d", VISUAL_LENGTH_m,
            new Color8Bit(MEASURED_COLOR_RED, MEASURED_COLOR_GREEN, MEASURED_COLOR_BLUE));
    private final Arm2d setpoint2d = new Arm2d("Arm/Setpoint2d", VISUAL_LENGTH_m,
            new Color8Bit(SETPOINT_COLOR_RED, SETPOINT_COLOR_GREEN, SETPOINT_COLOR_BLUE));

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
            System.out.println("Arm initialized.");
        }
        return instance;
    }

    public Arm() {
        super("Arm");
        this.armDrivingMotorConfig = new MotorConfig(MOTOR_ID)
                .withCanbus(CANBUS)
                .withFollower(FOLLOWER_ID, FOLLOWER_ALIGNMENT)
                .withInverted(INVERTED)
                .withBrake(BRAKE)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_A)
                .withFoc(FOC_ENABLED)
                .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO)

                .withPIDGains(kP, kI, kD, GRAVITY_TYPE)
                .withFFGains(kS, kV, kA, kG)
                .withMotionMagic(MM_CRUISE_VELOCITY_degps, MM_ACCELERATION_degps2, MM_JERK_degps3)
                .withSim(SIM_MOTOR, SIM_GEARING, SIM_MOI_kgm2)
                .withMeasurementUnit(MEASUREMENT_UNIT);

        this.armDrivingMotor_2 = new Motor("ArmMotorPair", this.armDrivingMotorConfig);

        setCommand(Command.IDLE);
    }

    @Override
    public void inputPeriodic() {
        armDrivingMotor_2.readInputs();
    }

    @Override
    protected void handle() {
        switch(getCommand()) {
            case DISABLED:
                armDrivingMotor_2.stop();
                break;

            case IDLE:
                armDrivingMotor_2.setVoltage(IDLE_VOLTS);
                break;

            case HOMING:
                
                break;

            case GO_TO_ANGLE:
                if (firstLoop()) {
                    setSubstate(TravelSubstate.MOVING);
                }
                switch((TravelSubstate)getSubstate()) {
                    case MOVING:
                        if (atTargetAngle(ANGLE_TOLERANCE_deg)) {
                            setSubstate(TravelSubstate.HOLDING);
                        }
                        armDrivingMotor_2.setMotionMagic(targetAngle_deg);
                        break;
                    case HOLDING:
                        if (!atTargetAngle(ANGLE_TOLERANCE_deg)) {
                            setSubstate(TravelSubstate.MOVING);
                        }
                        armDrivingMotor_2.setMotionMagic(targetAngle_deg);
                        break;
                }
                break;

            case MANUAL:
                armDrivingMotor_2.setVoltage(targetVolts);
                break;
        }
    }

    @Override
    protected void outputPeriodic() {
        measured2d.setAngle(getCurrentAngle());
        setpoint2d.setAngle(targetAngle_deg);
        measured2d.periodic();
        setpoint2d.periodic();

        Logger.recordOutput("Arm/Angle_deg", getCurrentAngle());
        Logger.recordOutput("Arm/TargetAngle_deg", targetAngle_deg);
    }

    public boolean atTargetAngle(double tol) {
        return Math.abs(getCurrentAngle() - targetAngle_deg) < tol;
    }

    public double getCurrentAngle() {
        return armDrivingMotor_2.getPosition();
    }   

    public void setTargetAngle(double angle_deg) {
        this.targetAngle_deg = angle_deg;
    }

    public void setTargetVolts(double volts) {
        this.targetVolts = volts;
    }

    public void goToVoltsManual(double volts) {
        setTargetVolts(volts);
        setCommand(Command.MANUAL);
    }

    public void goToAngle(double angle_deg) {
        setTargetAngle(angle_deg);
        setCommand(Command.GO_TO_ANGLE);
    }
}
