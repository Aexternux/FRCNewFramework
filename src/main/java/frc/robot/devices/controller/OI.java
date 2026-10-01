package frc.robot.devices.controller;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.util.Util;

public class OI extends XboxController {
    // config
    static {
        DR = new OI(0);
        XK = new XK80(1);
        DEADBAND_RADIUS = 0.01;
    }

    public static final OI DR;
    public static final XK80 XK;

    private static final double DEADBAND_RADIUS;

    public static double deadband(double jsValue) {
        double res = 0.0;

        if (Math.abs(jsValue) > DEADBAND_RADIUS) {
            res = Math.copySign(Util.remap(DEADBAND_RADIUS, 1.0, Math.abs(jsValue), 0.0, 1.0), jsValue);
        }

        return res;
    }

    private OI(int port) {
        super(port);
    }

    public boolean hasInput() {
        if (this.getAButton()) return true;
        if (this.getBButton()) return true;
        if (this.getXButton()) return true;
        if (this.getYButton()) return true;
        if (this.getStartButton()) return true;
        if (this.getBackButton()) return true;
        if (this.getPOV() != -1) return true;
        if (this.getLeftStickButton()) return true;
        if (this.getRightStickButton()) return true;
        if (this.getLeftBumperButton()) return true;
        if (this.getRightBumperButton()) return true;
        if (getLeftTriggerAxis() > 0.0) return true;
        if (getRightTriggerAxis() > 0.0) return true;
        if (getLeftY() != 0.0) return true;
        if (getLeftX() != 0.0) return true;
        if (getRightY() != 0.0) return true;
        if (getRightX() != 0.0) return true;
        return false;
    }
}
