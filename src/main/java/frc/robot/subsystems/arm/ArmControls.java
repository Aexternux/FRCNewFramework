package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import frc.robot.devices.controller.OI;
import frc.robot.util.Controls;

public class ArmControls extends Controls {
    private Arm arm;

    public ArmControls(Arm arm) {
        this.arm = arm;
    }

    public void update() {
        if (OI.DR.getRightTriggerAxis() > TRIGGER_THRESHOLD) {
            arm.goToAngle(RIGHT_TRIGGER_ANGLE_deg);
        } else if (OI.DR.getLeftTriggerAxis() > TRIGGER_THRESHOLD) {
            arm.goToAngle(LEFT_TRIGGER_ANGLE_deg);
        } else {
            arm.goToAngle(STOW_ANGLE_deg);
        }
    }
}
