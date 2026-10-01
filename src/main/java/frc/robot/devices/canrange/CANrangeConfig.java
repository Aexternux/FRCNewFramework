package frc.robot.devices.canrange;

public class CANrangeConfig {

    public int canId;
    public String canbus = "rio";

    public double proximityThreshold_m = 0.1;
    public double proximityHysteresis_m = 0.01;
    public double minSignalStrength = 2500.0;

    public CANrangeConfig(int canId) {
        this.canId = canId;
    }

    public CANrangeConfig withCanbus(String canbus) {
        this.canbus = canbus;
        return this;
    }

    public CANrangeConfig withProximity(double threshold_m, double hysteresis_m) {
        this.proximityThreshold_m = threshold_m;
        this.proximityHysteresis_m = hysteresis_m;
        return this;
    }

    public CANrangeConfig withMinSignalStrength(double strength) {
        this.minSignalStrength = strength;
        return this;
    }
}
