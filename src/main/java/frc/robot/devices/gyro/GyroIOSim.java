package frc.robot.devices.gyro;

/** Placeholder gyro simulation implementation. Drive currently uses wheel-delta heading in SIM. */
public class GyroIOSim implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        // Simulation gyro samples will be populated when a drivetrain gyro simulation is added.
    }
}
