package frc.robot.devices.motor;

import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.sim.PhysicsSim;

import java.util.ArrayList;
import java.util.List;

/**
 * The robot's own {@link MotorIOTalonFX}, running against the CAD physics simulator.
 *
 * <p>Nothing is re-implemented: the same TalonFX objects, configuration, control requests and
 * Motion Magic as on the robot, with Phoenix 6 simulating the Talon firmware. The only addition is
 * registering the Talons with {@link PhysicsSim}, which feeds the firmware's output voltage to the
 * physics and the physics' rotor position and velocity back to the Talons. If a mechanism works
 * here, the same code path drives it on the robot.
 */
public class MotorIOPhysicsSim extends MotorIOTalonFX {

    private final MotorConfig config;

    public MotorIOPhysicsSim(String name, MotorConfig config) {
        super(config);
        this.config = config;
        List<Boolean> opposed = new ArrayList<>();
        for (MotorConfig.FollowerConfig f : config.followers) {
            opposed.add(f.alignment() == MotorAlignmentValue.Opposed);
        }
        PhysicsSim.getInstance().registerMotor(name, "motor", -1, config.canbus, getTalon(), getFollowers(),
                opposed, config.inverted, config.brake, config.sensorToMechanismRatio);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        super.setBrakeMode(brake);
        PhysicsSim.getInstance().setBrake(getTalon(), brake);
    }

    public MotorConfig getConfig() {
        return config;
    }
}
