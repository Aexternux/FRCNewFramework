package frc.robot.subsystems;

import frc.robot.util.CustomTimer;
import frc.robot.util.PerfTracker;

import org.littletonrobotics.junction.Logger;


public abstract class SubsystemBase<C extends Enum<C>> {

    private final String name;
    protected final CustomTimer commandTimer = new CustomTimer();
    protected final CustomTimer substateTimer = new CustomTimer();

    private C command;
    private Enum<?> substate;
    private boolean firstLoop = true;
    private boolean substateFirstLoop = false;

    protected SubsystemBase(String name) {
        this.name = name;
    }

    public final void setCommand(C next) {
        if (command == null || command != next) {
            command = next;
            commandTimer.reset();
            substate = null;
            substateTimer.reset();
            firstLoop = true;
            substateFirstLoop = false;
        }
    }

    public final C getCommand() {
        return command;
    }

    public final boolean isCommand(C c) {
        return command == c;
    }

    protected final boolean firstLoop() {
        return firstLoop;
    }

    protected final void setSubstate(Enum<?> next) {
        if (substate == null || substate != next) {
            substate = next;
            substateTimer.reset();
            substateFirstLoop = true;
        }
    }

    public final Enum<?> getSubstate() {
        return substate;
    }

    protected final boolean isSubstate(Enum<?> s) {
        return substate == s;
    }


    protected final boolean substateInit() {
        if (substateFirstLoop) {
            substateFirstLoop = false;
            return true;
        }
        return false;
    }

    protected final boolean substateElapsed(double seconds) {
        return substateTimer.time() >= seconds;
    }

    protected abstract void inputPeriodic();

    protected abstract void handle();

    protected abstract void outputPeriodic();

    public final void periodic() {
        int id = PerfTracker.start(name);
        inputPeriodic();
        handle();
        firstLoop = false;
        outputPeriodic();
        PerfTracker.end(id);

        Logger.recordOutput(name + "/Command", command);
        Logger.recordOutput(name + "/CommandTimer", commandTimer.time());
        Logger.recordOutput(name + "/Substate", substate == null ? "NONE" : substate.name());
        Logger.recordOutput(name + "/SubstateTimer", substateTimer.time());
    }
}
