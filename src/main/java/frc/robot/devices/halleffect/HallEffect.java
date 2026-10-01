package frc.robot.devices.halleffect;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.Constants;

import org.littletonrobotics.junction.Logger;

public class HallEffect {

    public enum Mode {
        REAL,
        SIM
    }

    private final String name;
    private final HallEffectIO io;
    private final HallEffectIOInputsAutoLogged inputs = new HallEffectIOInputsAutoLogged();
    private final Debouncer debouncer;
    private final boolean debounceEnabled;

    private boolean detected = false;

    public HallEffect(String name, HallEffectConfig config, Mode mode) {
        this.name = name;
        if (mode == Mode.REAL) {
            this.io = new HallEffectIODIO(config);
        } else {
            this.io = new HallEffectIOSim(config);
        }
        this.debounceEnabled = config.debounceTime > 0.0;
        this.debouncer = new Debouncer(config.debounceTime, config.debounceType);
    }

    public HallEffect(String name, HallEffectConfig config) {
        this(name, config, Constants.currentMode == Constants.Mode.REAL ? Mode.REAL : Mode.SIM);
    }

    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
        detected = debounceEnabled ? debouncer.calculate(inputs.detected) : inputs.detected;
        Logger.recordOutput(name + "/Detected", detected);
    }

    public boolean get() {
        return detected;
    }

    public boolean getRaw() {
        return inputs.detected;
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public void setSimState(boolean detected) {
        io.setSimDetected(detected);
    }
}
