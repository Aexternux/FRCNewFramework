package frc.robot.subsystems.tracking;

public class TrackingLocation {

    double tz;
    double tx;

    public TrackingLocation(double tx, double tz) {
        this.tz = tz;
        this.tx = tx;
    }

    public double getTx() {
        return tx;
    }

    public double getTz() {
        return tz;
    }
    
}