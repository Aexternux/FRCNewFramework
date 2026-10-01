package frc.robot.devices.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;

import org.littletonrobotics.junction.Logger;

/**
 * Mechanism-facing gyro device wrapper.
 *
 * <p>This class owns mode-specific IO selection, AdvantageKit input logging, and the small API
 * consumed by Drive. Drive retains ownership of how yaw samples are used for odometry.
 */
public class Gyro {
    private final String name;
    private final GyroIO io;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public Gyro(String name) {
        this.name = name;
        io = switch (Constants.currentMode) {
            case REAL -> new GyroIOPigeon2();
            // Drive deliberately uses wheel-delta heading until the gyro simulation supplies
            // synchronized odometry samples.
            case SIM, REPLAY -> new GyroIO() {};
        };
    }

    /** Reads and logs the most recent gyro inputs. Call once per robot loop. */
    public void readInputs() {
        io.updateInputs(inputs);
        Logger.processInputs(name, inputs);
    }

    /** Sets the reported yaw angle in degrees. */
    public void zero(double angle_deg) {
        io.zero(angle_deg);
    }

    public boolean isConnected() {
        return inputs.connected;
    }

    public Rotation2d getYaw() {
        return inputs.yaw_Rot2d;
    }

    public Rotation2d getPitch() {
        return inputs.pitch_Rot2d;
    }

    public Rotation2d getRoll() {
        return inputs.roll_Rot2d;
    }

    public double getYawVelocity_radps() {
        return inputs.yawVel_radps;
    }

    /** Returns synchronized high-rate timestamps used by Drive's odometry estimator. */
    public double[] getOdometryTimestamps_s() {
        return inputs.odometryYawTimestamps;
    }

    /** Returns yaw samples corresponding to {@link #getOdometryTimestamps_s()}. */
    public Rotation2d[] getOdometryYawPositions() {
        return inputs.odometryYawPositions;
    }
}
