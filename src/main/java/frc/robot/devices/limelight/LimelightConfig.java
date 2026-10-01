package frc.robot.devices.limelight;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Builder-style configuration for a {@link Limelight} device. */
public class LimelightConfig {

    /** NetworkTables name of the camera, e.g. {@code "limelight-sean"}. */
    public final String name;

    /** Transform from the robot origin to the camera lens. Used for the simulated camera. */
    public Transform3d robotToCamera = new Transform3d();

    /**
     * MegaTag1 is only trusted when the average tag distance is below this value (meters); past it
     * the device falls back to MegaTag2, which fuses the gyro heading.
     */
    public double mt1MaxDistance_m = 0.2;

    /** Sim only: the camera won't detect tags farther than this. */
    public double maxSightRange_m = Double.POSITIVE_INFINITY;

    public LimelightConfig(String name) {
        this.name = name;
    }

    public LimelightConfig withRobotToCamera(Transform3d robotToCamera) {
        this.robotToCamera = robotToCamera;
        return this;
    }

    public LimelightConfig withRobotToCamera(
            double x_m, double y_m, double z_m,
            double roll_deg, double pitch_deg, double yaw_deg) {
        this.robotToCamera = new Transform3d(
                new Translation3d(x_m, y_m, z_m),
                new Rotation3d(
                        Units.degreesToRadians(roll_deg),
                        Units.degreesToRadians(pitch_deg),
                        Units.degreesToRadians(yaw_deg)));
        return this;
    }

    public LimelightConfig withMegaTag1MaxDistance(double distance_m) {
        this.mt1MaxDistance_m = distance_m;
        return this;
    }

    // Builder setter for the sim sight-range cap (chainable: new LimelightConfig(...).withMaxSightRange(3)).
    public LimelightConfig withMaxSightRange(double range_m) {
        this.maxSightRange_m = range_m;
        return this;
    }
}
