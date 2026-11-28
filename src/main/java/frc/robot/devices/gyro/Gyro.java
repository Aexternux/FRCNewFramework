package frc.robot.devices.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        switch (Constants.currentMode) {
            case REAL:
                io = new GyroIOPigeon2();
                break;
            case SIM:
                // The physics simulator's Pigeon2 reads the robot's real heading. Without it Drive
                // falls back to wheel-delta heading.
                if (Constants.physicsSim) {
                    io = new GyroIOPhysicsSim();
                } else {
                    io = new GyroIO() {};
                }
                break;
            case REPLAY:
                io = new GyroIO() {};
                break;
            default:
                io = new GyroIO() {};
                break;
        }
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

    /** Returns measured acceleration in the gyro's frame (X forward, Y left when mounted square). */
    public Translation2d getAccel_mps2() {
        return new Translation2d(inputs.accelX_mps2, inputs.accelY_mps2);
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
