package frc.robot.devices.controller;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.GenericHID;

public class XK80 extends GenericHID {

    public static final int WIDTH = 10;
    public static final int HEIGHT = 8;

    public XK80(final int port) {
        super(port);
        HAL.report(tResourceType.kResourceType_XboxController, port + 1);
    }

    public boolean get(int x, int y) {
        if (x < 5) {
            if (x == 2) {
                x = 3;
            } else if (x == 3) {
                x = 2;
            }
            int res = (int) ((getRawAxis(x) + 0.7890625) * 5.07);
            return res == y + 1;
        } else if (x < 9) {
            int res = 0, colshift = (1 - x % 2) * 4, rowshift = ((x - 6) > 0 ? 8 : 0);
            return getRawButton((x * 8 + y) - 39);
        } else {
            int pov = getPOV();
            if (pov < 0) return false;
            return (pov / 45) == y;
        }
    }

    public boolean hasInput() {
        for (int x = 0; x < WIDTH - 1; x++) for (int y = 0; y < HEIGHT - 1; y++) if (get(x, y)) return true;
        return false;
    }
}
