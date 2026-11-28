package frc.robot.devices.gyro;

import frc.robot.generated.TunerConstants;
import frc.robot.sim.PhysicsSim;

/**
 * The robot's own {@link GyroIOPigeon2}, running against the CAD physics simulator: the simulated
 * Pigeon2 reads the physics robot's actual heading and turn rate (wheel slip included), through
 * {@link PhysicsSim}.
 */
public class GyroIOPhysicsSim extends GyroIOPigeon2 {

    public GyroIOPhysicsSim() {
        super();
        PhysicsSim.getInstance().registerGyro(TunerConstants.DrivetrainConstants.CANBusName, getPigeon());
    }
}
