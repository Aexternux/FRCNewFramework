package frc.robot;

import frc.robot.application.superstructure.SS;
import frc.robot.application.superstructure.SS.Flag;
import frc.robot.devices.controller.OI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.drive.SwerveInput;
import frc.robot.util.Controls;
import frc.robot.util.Util;


public class ControlScheme extends Controls{
    private static final double ROTATION_INPUT_SCALE = 0.5;
    private static final int GYRO_ZERO_POV_deg = 180;
    private static final double TRACKING_TRIGGER_THRESHOLD = 0.5;

    private final SS ss;
    private final Drive drive;

    public ControlScheme(SS ss, Drive drive) {
        super();
        this.ss = ss;
        this.drive = drive;
    }

    public void init() {
        drive.setCommand(PathingMode.FIELD_RELATIVE);
        drive.setTrackingRequested(false);
        System.out.println("Controls initialized");
    }

    public void update() {
        double x_ = OI.deadband(-OI.DR.getLeftY());
        double y_ = OI.deadband(-OI.DR.getLeftX());
        double w_ = ROTATION_INPUT_SCALE * -Util.sqInput(OI.deadband(OI.DR.getRightX()));
        double throttle = Util.sqInput(
                1.0 - OI.deadband(Math.max(OI.DR.getLeftTriggerAxis(), OI.DR.getRightTriggerAxis())));

        if (OI.DR.getPOV() == GYRO_ZERO_POV_deg) {
            drive.zeroGyro();
        }
        drive.setInput(new SwerveInput(x_, y_, w_, throttle));

        if (OI.DR.getRightTriggerAxis() > TRACKING_TRIGGER_THRESHOLD) {
            drive.setTrackingRequested(true);
        } else {
            drive.setTrackingRequested(false);
        }

        ss.set(Flag.HOME, OI.DR.getBackButton());
        ss.set(Flag.MANUAL_UP, OI.DR.getAButton());
        ss.set(Flag.MANUAL_DOWN, OI.DR.getBButton());
        ss.set(Flag.SCORE_LOW, OI.DR.getXButton());
        ss.set(Flag.SCORE_HIGH, OI.DR.getYButton());
    }
}
