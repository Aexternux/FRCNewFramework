package frc.robot.devices.motor;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

    @AutoLog
    public static class MotorIOInputs {
        public boolean connected = false;
        public double position = 0.0;
        public double velocity = 0.0; 
        public double acceleration = 0.0; 
        public double appliedVolts = 0.0;
        public double supplyCurrent = 0.0;
        public double statorCurrent = 0.0;
        public double tempC = 0.0;
    }

    public default void updateInputs(MotorIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setOpenLoop(double dutyCycle) {}

    public default void setVelocity(double velocity) {}

    public default void setPositionVoltage(double position) {}

    public default void setMotionMagic(double position) {}

    public default void setMotionMagicVelocity(double velocity) {}

    public default void stop() {}

    public default void setBrakeMode(boolean brake) {}

    public default void zeroPosition(double position) {}
}
