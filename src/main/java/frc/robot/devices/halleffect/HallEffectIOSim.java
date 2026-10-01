package frc.robot.devices.halleffect;

public class HallEffectIOSim implements HallEffectIO {

    private boolean detected = false;

    public HallEffectIOSim(HallEffectConfig config) {}

    @Override
    public void updateInputs(HallEffectIOInputs inputs) {
        inputs.connected = true;
        inputs.detected = detected;
    }

    @Override
    public void setSimDetected(boolean detected) {
        this.detected = detected;
    }
}
