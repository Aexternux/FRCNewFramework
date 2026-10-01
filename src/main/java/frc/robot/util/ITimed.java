package frc.robot.util;

public interface ITimed {

    public abstract double time();

    public abstract boolean active();

    public abstract void reset();

    public abstract boolean after(double seconds);

    public abstract boolean before(double seconds);

    public abstract boolean between(double earlier, double later);

    public abstract double alpha(double earlier, double later);
}
