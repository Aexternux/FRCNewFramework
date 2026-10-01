package frc.robot.devices.canrange;

public class CANrangeIOSim implements CANrangeIO {

    private final CANrangeConfig config;
    private double distance_m;

    public CANrangeIOSim(CANrangeConfig config) {
        this.config = config;
        this.distance_m = config.proximityThreshold_m * 2.0;
    }

    @Override
    public void updateInputs(CANrangeIOInputs inputs) {
        inputs.connected = true;
        inputs.distance_m = distance_m;
        inputs.isDetected = distance_m <= config.proximityThreshold_m;
        inputs.signalStrength = config.minSignalStrength * 2.0;
        inputs.ambientSignal = 0.0;
    }

    @Override
    public void setSimDistance(double distance_m) {
        this.distance_m = distance_m;
    }
}
