package frc.robot.application.superstructure;

import frc.robot.ControlScheme;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

public class SS extends SubsystemBase<SS.Command> {

    public enum Flag {
        HOME,
        SCORE_LOW,
        SCORE_HIGH,
        MANUAL_UP,
        MANUAL_DOWN
    }

    public enum Command {
        IDLE,
        HOMING,
        STOWING,
        SCORING,
        MANUAL
    }

    private enum Scoring {
        RAISING,
        SETTLING,
        READY
    }

    private static final double MANUAL_VOLTS = 2.0;
    private static final double SCORE_LOW_HEIGHT_m = 1.2;
    private static final double SETTLE_TIME_s = 0.2;

    private static SS instance;

    private final EnumSet<Flag> flags = EnumSet.noneOf(Flag.class);

    private final Elevator elevator;
    private final Drive drive;

    private double scoreTarget_m = ElevatorConstants.MIN_HEIGHT_m;

    public static SS getInstance() {
        if (instance == null) {
            instance = new SS();
        }
        return instance;
    }

    private SS() {
        super("Superstructure");
        elevator = Elevator.getInstance();
        drive = Drive.getInstance();
        setCommand(Command.IDLE);
    }


    public void enable(Flag flag) {
        flags.add(flag);
    }

    public void disable(Flag flag) {
        flags.remove(flag);
    }

    public void set(Flag flag, boolean active) {
        if (active) {
            flags.add(flag);
        } else {
            flags.remove(flag);
        }
    }

    public void toggle(Flag flag) {
        set(flag, !has(flag));
    }

    public boolean has(Flag flag) {
        return flags.contains(flag);
    }

    @Override
    protected void inputPeriodic() {
        
    }

    @Override
    protected void handle() {
        if (has(Flag.HOME)) {
            setCommand(Command.HOMING);
        } else if (has(Flag.MANUAL_UP) || has(Flag.MANUAL_DOWN)) {
            setCommand(Command.MANUAL);
        } else if (has(Flag.SCORE_HIGH)) {
            scoreTarget_m = ElevatorConstants.MAX_HEIGHT_m;
            setCommand(Command.SCORING);
        } else if (has(Flag.SCORE_LOW)) {
            scoreTarget_m = SCORE_LOW_HEIGHT_m;
            setCommand(Command.SCORING);
        } else {
            setCommand(Command.STOWING);
        }

        switch (getCommand()) {
            case HOMING:
                elevator.home();
                break;
            case MANUAL:
                elevator.manual(has(Flag.MANUAL_UP) ? MANUAL_VOLTS : -MANUAL_VOLTS);
                break;
            case SCORING:
                handleScoring();
                break;
            case STOWING:
                elevator.trackToHeight(ElevatorConstants.MIN_HEIGHT_m);
                break;
            case IDLE:
                break;
        }
    }

    private void handleScoring() {
        if (firstLoop()) {
            setSubstate(Scoring.RAISING);
        }

        switch ((Scoring) getSubstate()) {
            case RAISING:
                elevator.trackToHeight(scoreTarget_m);
                if (elevator.atTarget(ElevatorConstants.TOLERANCE_m)) {
                    setSubstate(Scoring.SETTLING);   
                }
                break;

            case SETTLING:
                elevator.trackToHeight(scoreTarget_m);
                if (!elevator.atTarget(ElevatorConstants.TOLERANCE_m)) {
                    setSubstate(Scoring.RAISING);    
                } else if (substateElapsed(SETTLE_TIME_s)) {
                    setSubstate(Scoring.READY);    
                }
                break;

            case READY:
                elevator.trackToHeight(scoreTarget_m);
                if (!elevator.atTarget(ElevatorConstants.TOLERANCE_m)) {
                    setSubstate(Scoring.RAISING);   
                }
                break;
        }
    }

    @Override
    protected void outputPeriodic() {
        String[] active = flags.stream().map(Enum::name).toArray(String[]::new);
        Logger.recordOutput("Superstructure/Flags", active);
    }
}
