package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color8Bit;

import frc.robot.Robot;
import frc.robot.devices.halleffect.HallEffect;
import frc.robot.devices.halleffect.HallEffectConfig;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.util.Util;

import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase<Elevator.Command> {

    public enum Command {
        DISABLED,
        IDLE,
        HOMING,
        GO_TO_HEIGHT,
        MANUAL
    }

    private enum Homing {
        SEEKING,
        SETTLED
    }

    private enum Travel {
        MOVING,
        HOLDING
    }

    private static Elevator instance;

    private final Motor motor;
    private final HallEffect hallEffect = new HallEffect(
            "Elevator/HallEffect",
            new HallEffectConfig(HALL_EFFECT_CHANNEL)
                    .withInverted(HALL_INVERTED)
                    .withDebounce(HALL_DEBOUNCE_s, HALL_DEBOUNCE_TYPE));
    private final Elevator2d measured2d = new Elevator2d("Elevator/Measured2d", new Color8Bit(200, 0, 0));
    private final Elevator2d setpoint2d = new Elevator2d("Elevator/Setpoint2d", new Color8Bit(100, 100, 100));

    private double targetHeight_m = MIN_HEIGHT_m;
    private double voltsTarget = 0.0;
    private boolean hallDetected = false;
    private boolean zeroed = false;

    public static Elevator getInstance() {
        if (instance == null) {
            instance = new Elevator();
            System.out.println("Elevator initialized.");
        }
        return instance;
    }

    private Elevator() {
        super("Elevator");

        MotorConfig config = new MotorConfig(MOTOR_ID)
                .withCanbus(CANBUS)
                .withFollower(FOLLOWER_ID, FOLLOWER_OPPOSE)
                .withInverted(INVERTED)
                .withBrake(BRAKE)
                .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_A)
                .withSensorToMechanismRatio(METERS_TO_ROTATIONS)
                .withMeasurementUnit(MotorConfig.MeasurementUnit.METERS)
                
                .withFFGains(kS, kV, kA, kG)
                .withPIDGains(kP, kI, kD, GRAVITY)
                .withMotionMagic(MM_CRUISE_VELOCITY, MM_ACCELERATION, MM_JERK)
                .withSim(SIM_MOTOR, METERS_TO_ROTATIONS, SIM_MOI);

        motor = new Motor("Elevator/Motor", config);

        setCommand(Command.IDLE);
    }

    @Override
    protected void inputPeriodic() {
        motor.readInputs();
        hallEffect.readInputs();
        hallDetected = hallEffect.get();
    }

    @Override
    protected void handle() {
        switch (getCommand()) {
            case DISABLED:
                motor.stop();
                break;

            case IDLE:
                motor.setVoltage(0.0);
                break;

            case HOMING:
                if (firstLoop()) {
                    setSubstate(zeroed ? Homing.SETTLED : Homing.SEEKING);
                }
                switch ((Homing) getSubstate()) {
                    case SEEKING:
                        motor.setVoltage(HOMING_VOLTS);
                        if (Robot.isSimulation() || hallDetected) {
                            motor.zeroPosition(MIN_HEIGHT_m);
                            zeroed = true;
                            setSubstate(Homing.SETTLED);   
                        }
                        break;
                    case SETTLED:
                        setCommand(Command.IDLE);          
                        break;
                }
                break;

            case GO_TO_HEIGHT:
                if (firstLoop()) {
                    setSubstate(Travel.MOVING);
                }
                switch ((Travel) getSubstate()) {
                    case MOVING:
                        motor.setMotionMagic(targetHeight_m);
                        if (atTarget(TOLERANCE_m)) {
                            setSubstate(Travel.HOLDING);   
                        }
                        break;
                    case HOLDING:
                        motor.setMotionMagic(targetHeight_m);
                        if (!atTarget(TOLERANCE_m)) {
                            setSubstate(Travel.MOVING);    
                        }
                        break;
                }
                break;

            case MANUAL:
                motor.setVoltage(voltsTarget);
                break;
        }
    }

    @Override
    protected void outputPeriodic() {
        measured2d.setHeight(getHeight());
        setpoint2d.setHeight(targetHeight_m);
        measured2d.periodic();
        setpoint2d.periodic();

        Logger.recordOutput("Elevator/Height_m", getHeight());
        Logger.recordOutput("Elevator/Velocity_mps", motor.getVelocity());
        Logger.recordOutput("Elevator/TargetHeight_m", targetHeight_m);
        Logger.recordOutput("Elevator/Zeroed", isZeroed());
    }


    public void idle() {
        setCommand(Command.IDLE);
    }

    public void home() {
        setCommand(Command.HOMING);
    }

    public void trackToHeight(double height_m) {
        targetHeight_m = MathUtil.clamp(height_m, MIN_HEIGHT_m, MAX_HEIGHT_m);
        setCommand(Command.GO_TO_HEIGHT);
    }

    public void manual(double volts) {
        voltsTarget = volts;
        setCommand(Command.MANUAL);
    }

    public double getHeight() {
        return motor.getPosition();
    }

    public double getTargetHeight() {
        return targetHeight_m;
    }

    public boolean atTarget(double tol) {
        return Util.inRange(targetHeight_m - getHeight(), tol);
    }

    public boolean isZeroed() {
        return zeroed || Robot.isSimulation();
    }
}
