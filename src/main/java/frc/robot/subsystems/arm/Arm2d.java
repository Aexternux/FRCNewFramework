package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** A Mechanism2d visualization of a single rotating arm. Angles are in degrees. */
public class Arm2d {
    private final String name;
    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismLigament2d arm;

    public Arm2d(String name, Color8Bit color) {
        this(name, VISUAL_LENGTH_m, color);
    }

    public Arm2d(String name, double lengthMeters, Color8Bit color) {
        this.name = name;
        mechanism = new LoggedMechanism2d(VISUAL_CANVAS_WIDTH_m, VISUAL_CANVAS_HEIGHT_m);
        LoggedMechanismRoot2d root = mechanism.getRoot("Arm Pivot", VISUAL_PIVOT_X_m, VISUAL_PIVOT_Y_m);
        arm = root.append(new LoggedMechanismLigament2d(
                "Arm", lengthMeters, STOW_ANGLE_deg, VISUAL_LIGAMENT_THICKNESS_px, color));
    }

    public void setAngle(double angleDegrees) {
        arm.setAngle(angleDegrees);
    }

    public void periodic() {
        SmartDashboard.putData(name, mechanism);
        Logger.recordOutput(name, mechanism);
    }
}
