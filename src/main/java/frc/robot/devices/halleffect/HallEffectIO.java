package frc.robot.devices.halleffect;

import org.littletonrobotics.junction.AutoLog;

public interface HallEffectIO {

    @AutoLog
    public static class HallEffectIOInputs {
        public boolean connected = false;
        public boolean detected = false;
    }

    public default void updateInputs(HallEffectIOInputs inputs) {}

    public default void setSimDetected(boolean detected) {}
}
