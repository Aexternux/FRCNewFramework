package frc.robot.application.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.application.auto.ActionList;
import frc.robot.application.auto.Waylist;
import frc.robot.constants.Enums.StartLocation;

/**
 * Drivetrain-only ports of frc2k26's autonomous paths. Mechanism intentions from the source
 * project are intentionally omitted because this robot has no matching intake/shooter interface.
 */
public final class ReferenceAutos {
    private static final double MAX_VEL_mps = 4.8;
    private static final double TRANSLATION_kP = 2.5;
    private static final double ROTATION_kP = 0.2;
    private static final double INTAKE_MAX_VEL_mps = 2.5;

    private ReferenceAutos() {}

    public static ActionList leftAuto1() {
        ActionList auto = new ActionList("LEFT_AUTO_1", StartLocation.LEFT,
                ActionList.PathingStyle.CONTINUOUS).startActions();
        auto.driveToPose(w(4.407, 7.580, 180, .30), MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(5.000, 7.400, 180, .30).append(p(7.000, 7.300, 180), .30)
                .append(p(7.657, 7.200, 270), .30), INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(7.657, 5.535, 270, .15).append(p(7.657, 4.935, 270), .15),
                INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(6.409, 5.993, 135, .15).append(p(7.000, 5.993, 0), .15)
                .append(p(6.000, 5.993, 0), .20), INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(3.000, 5.993, 0, .10), INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.waitFor(5.5);
        auto.driveToPose(w(2.800, 7.400, 0, .10), MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(5.900, 7.300, 270, .50), INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(5.657, 6.135, 270, .30).append(p(5.657, 4.235, 270), .30),
                INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(6.409, 5.993, 135, .30).append(p(7.000, 5.993, 0), .30)
                .append(p(6.000, 5.993, 0), .25), MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(3.000, 5.993, 0, .20), INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        return auto.waitFor(11.5);
    }

    public static ActionList rightAuto1() {
        ActionList auto = new ActionList("RIGHT_AUTO_1", StartLocation.RIGHT,
                ActionList.PathingStyle.CONTINUOUS).startActions();
        auto.driveToPose(w(5.000, .500, 180, .30).append(p(7.000, .500, 180), .30)
                .append(p(7.657, .800, 90), .30), INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(7.657, 2.535, 90, .15).append(p(7.657, 3.135, 90), .15),
                INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(6.409, 2.377, 90, .15).append(p(7.000, 2.377, 45), .15)
                .append(p(6.000, 2.377, 0), .20), MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(2.300, 2.377, 0, .10), MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.waitFor(5.5);
        auto.driveToPose(w(2.800, .540, 0, .20), MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(6.000, .600, 0, .50), INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(5.657, 1.000, 90, .15).append(p(5.657, 3.550, 90), .15),
                INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(6.409, 2.377, 90, .15).append(p(7.000, 2.377, 45), .15)
                .append(p(6.000, 2.377, 0), .20), INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(2.300, 2.377, 0, .10), INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        return auto.waitFor(12.0);
    }

    /** Drivetrain-only port of frc2k26's left disrupt path. */
    public static ActionList leftDisrupt() {
        ActionList auto = new ActionList("LEFT_DISRUPT", StartLocation.LEFT,
                ActionList.PathingStyle.CONTINUOUS).startActions();
        auto.waitFor(.5);
        auto.driveToPose(w(4.407, 7.580, 180, .30), MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(5.000, 7.400, 180, .30).append(p(7.000, 7.300, 180), .30)
                .append(p(8.000, 7.200, 270), .30), INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.waitFor(3.0);
        auto.driveToPose(w(10.691, 5.800, -45, .30), MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.waitFor(3.0);
        auto.driveToPose(w(10.691, 3.200, 270, .30).append(p(10.691, 4.800, 270), .30)
                .append(p(7.809, 2.600, 180), .35), MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        return auto.waitFor(20.0);
    }

    /** Drivetrain-only port of frc2k26's right disrupt path. */
    public static ActionList rightDisrupt() {
        ActionList auto = new ActionList("RIGHT_DISRUPT", StartLocation.RIGHT,
                ActionList.PathingStyle.CONTINUOUS).startActions();
        auto.waitFor(.5);
        auto.driveToPose(w(4.407, .500, 180, .30), MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.driveToPose(w(5.000, .500, 180, .30).append(p(7.000, .600, 180), .30)
                .append(p(8.000, .700, 90), .30), INTAKE_MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.waitFor(3.0);
        auto.driveToPose(w(10.691, 1.800, 45, .30), MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        auto.waitFor(3.0);
        auto.driveToPose(w(10.691, 4.600, 90, .30).append(p(7.809, 6.400, 180), .35),
                MAX_VEL_mps, TRANSLATION_kP, ROTATION_kP);
        return auto.waitFor(20.0);
    }

    private static Waylist w(double x_m, double y_m, double heading_deg, double tolerance_m) {
        return new Waylist(p(x_m, y_m, heading_deg), tolerance_m);
    }

    private static Pose2d p(double x_m, double y_m, double heading_deg) {
        return new Pose2d(x_m, y_m, Rotation2d.fromDegrees(heading_deg));
    }
}
