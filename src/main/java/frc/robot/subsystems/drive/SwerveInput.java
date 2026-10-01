// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.Util;

/** Add your docs here. */
public class SwerveInput {
    public static final double DONT_CARE = -7558;

    public static final SwerveInput ZERO = new SwerveInput(0, 0, 0);
    public static final SwerveInput FORWARD = new SwerveInput(1, 0, 0);
    public static final SwerveInput REVERSE = new SwerveInput(-1, 0, 0);
    public static final SwerveInput LEFT = new SwerveInput(0, 1, 0);
    public static final SwerveInput RIGHT = new SwerveInput(0, -1, 0);
    public static final SwerveInput CW = new SwerveInput(0, 0, -1);
    public static final SwerveInput CCW = new SwerveInput(0, 0, 1);

    public double xi, yi, wi, throttle, theta_r;

    public SwerveInput(double x, double y, double w, double throttle, double theta_r) {
        this.xi = Util.limit(x, 1.0);
        this.yi = Util.limit(y, 1.0);
        this.wi = Util.limit(w, 1.0);
        this.throttle = Util.limit(throttle, 0.0, 1.0);
        this.theta_r = w;
    }

    public SwerveInput(double x, double y, double w, double throttle) {
        this(x, y, w, throttle, DONT_CARE);
    }

    public SwerveInput(double x, double y, double w) {
        this(x, y, w, 1.0);
    }

    public SwerveInput(ChassisSpeeds speeds) {
        this(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    public SwerveInput(SwerveInput other) {
        xi = Util.limit(other.xi, 1.0);
        yi = Util.limit(other.yi, 1.0);
        wi = Util.limit(other.wi, 1.0);
        throttle = Util.limit(other.throttle, 0.0, 1.0);
        theta_r = other.theta_r;
    }

    /**
     * Only updates theta_r if we care about it
     *
     * @param other
     */
    public void set(SwerveInput other) {
        this.xi = Util.limit(other.xi, 1.0);
        this.yi = Util.limit(other.yi, 1.0);
        this.wi = Util.limit(other.wi, 1.0);
        this.throttle = Util.limit(other.throttle, 0.0, 1.0);
        // Truncates to subtract from parameter to keep theta_r between -1.0 and 1.0
        int downsample = (int) other.theta_r;
        this.theta_r = other.theta_r == DONT_CARE ? theta_r : (other.theta_r - downsample);
    }
}
