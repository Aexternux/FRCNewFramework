package frc.robot.devices.halleffect;

import edu.wpi.first.wpilibj.DigitalInput;

public class HallEffectIODIO implements HallEffectIO {

    private final HallEffectConfig config;
    private final DigitalInput input;

    public HallEffectIODIO(HallEffectConfig config) {
        this.config = config;
        this.input = new DigitalInput(config.channel);
    }

    @Override
    public void updateInputs(HallEffectIOInputs inputs) {
        boolean raw = input.get();
        inputs.connected = true;
        inputs.detected = config.inverted ? !raw : raw;
    }
}
