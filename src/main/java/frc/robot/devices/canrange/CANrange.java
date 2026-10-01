package frc.robot.devices.canrange;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.Constants;

public class CANrange {

    public enum Mode {
        REAL,
        SIM
    }

    private final String name;
    private final CANrangeIO io;
    private final CANrangeIOInputsAutoLogged inputs = new CANrangeIOInputsAutoLogged();

    public CANrange(String name, CANrangeConfig config, Mode mode) {
        this.name = name;
        if (mode == Mode.REAL) {
            this.io = new CANrangeIOReal(config);
        } else {
            this.io = new CANrangeIOSim(config);
        }
    }

    public CANrange(String name, CANrangeConfig config) {
        this(name, config, Constants.currentMode == Constants.Mode.REAL ? Mode.REAL : Mode.SIM);
    }

    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    public double getDistance() {
        return inputs.distance_m;
    }

    public boolean isDetected() {
        return inputs.isDetected;
    }

    public double getSignalStrength() {
        return inputs.signalStrength;
    }

    public double getAmbientSignal() {
        return inputs.ambientSignal;
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public void setSimDistance(double distance_m) {
        io.setSimDistance(distance_m);
    }
}
