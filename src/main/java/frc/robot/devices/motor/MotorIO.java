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

    /** Motion Magic velocity with a per-request acceleration; 0 uses the configured value. */
    public default void setMotionMagicVelocity(double velocity, double acceleration) {
        setMotionMagicVelocity(velocity);
    }

    /** Motion Magic velocity with an extra voltage feedforward added to the closed-loop output. */
    public default void setMotionMagicVelocity(double velocity, double acceleration,
            double feedForwardVolts) {
        setMotionMagicVelocity(velocity, acceleration);
    }

    public default void stop() {}

    public default void setBrakeMode(boolean brake) {}

    public default void zeroPosition(double position) {}
}
