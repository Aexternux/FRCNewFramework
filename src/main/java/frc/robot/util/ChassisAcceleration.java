package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;

public class ChassisAcceleration {

    public static ChassisSpeeds fromChassisSpeeds(ChassisSpeeds current, ChassisSpeeds next, double delta_s) {
        return next.minus(current).div(delta_s);
    }

    public static ChassisSpeeds fromAcceleration(ChassisSpeeds speed, ChassisSpeeds acc, double delta_s) {
        return acc.times(delta_s).plus(speed);
    }

    public static double magnitude(ChassisSpeeds s) {
        return Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond);
    }

    public static double angle(ChassisSpeeds s) {
        return Math.atan2(s.vyMetersPerSecond, s.vxMetersPerSecond);
    }

    public static ChassisSpeeds calculate(ChassisSpeeds prev, ChassisSpeeds current, double delta_s){
        return current.minus(prev).div(delta_s);
    }
}
