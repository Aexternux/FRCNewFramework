package frc.robot.application.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.application.auto.ActionList.Action;
import frc.robot.application.superstructure.SS;
import frc.robot.application.superstructure.SS.Flag;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;

/** Runs one {@link ActionList} using the drivetrain pose follower. */
public class Auto extends SubsystemBase<AutoState> {
    private static final double LOOKAHEAD_m = 1.2;
    private static final double COMPLETION_HEADING_TOLERANCE_rad = Math.toRadians(5.0);
    private static final double FULL_TURN_deg = 180.0;
    private static final double NO_COMPLETION_START_s = -1.0;

    private final Drive drive = Drive.getInstance();
    private final SS superstructure = SS.getInstance();
    private final ActionList actions;
    private double completionStart_s = NO_COMPLETION_START_s;
    private boolean waylistsFlipped = false;

    public Auto(ActionList actions) {
        super("Auto");
        this.actions = actions;
        setCommand(AutoState.IDLE);
    }

    /** Resets path progress and odometry, then starts the first action. */
    public void start() {
        actions.reset();
        // Applying the 180-degree field transform twice restores blue coordinates. Tracking the
        // applied alliance keeps a routine correct if DriverStation reports alliance after startup.
        boolean redAlliance = Constants.isRedAlliance();
        if (redAlliance != waylistsFlipped) {
            actions.transformWaylists(pose -> new Pose2d(
                    Constants.FIELD_LENGTH_M - pose.getX(), Constants.FIELD_WIDTH_M - pose.getY(),
                    pose.getRotation().rotateBy(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(FULL_TURN_deg))));
            waylistsFlipped = redAlliance;
        }
        resetStartingPose();
        startAction();
    }

    private void resetStartingPose() {
        Pose2d blueStartPose = actions.startPose();
        // resetGyroAndPose() handles the alliance heading offset itself, so only mirror x/y here.
        Pose2d startPose = Constants.isRedAlliance()
                ? new Pose2d(Constants.FIELD_LENGTH_M - blueStartPose.getX(),
                        Constants.FIELD_WIDTH_M - blueStartPose.getY(), blueStartPose.getRotation())
                : blueStartPose;
        drive.resetGyroAndPose(startPose.getRotation().getDegrees(), startPose);
        Logger.recordOutput("Auto/StartPose", startPose);
    }

    public void stop() {
        clearSuperstructureFlags();
        drive.setCommand(PathingMode.DISABLED);
        setCommand(AutoState.DISABLED);
    }

    @Override
    protected void inputPeriodic() {}

    @Override
    protected void handle() {
        if (DriverStation.isDisabled()) {
            stop();
            return;
        }

        switch (getCommand()) {
            case DRIVE -> handleDrive();
            case WAIT -> {
                Action waitAction = actions.currentAction();
                if (waitAction == null || commandTimer.time() >= waitAction.timeout_s) advance();
            }
            case FINISHED, IDLE, DISABLED -> { }
        }
    }

    private void startAction() {
        Action action = actions.currentAction();
        completionStart_s = NO_COMPLETION_START_s;
        commandTimer.reset();
        if (action == null) {
            clearSuperstructureFlags();
            drive.setCommand(PathingMode.DISABLED);
            setCommand(AutoState.FINISHED);
            return;
        }

        applySuperstructureFlags(action);
        if (action.type == ActionList.ActionType.DRIVE) {
            setCommand(AutoState.DRIVE);
            commandDrive(action);
        } else {
            drive.setCommand(PathingMode.DISABLED);
            setCommand(AutoState.WAIT);
        }
    }

    private void handleDrive() {
        Action action = actions.currentAction();
        if (action == null || commandTimer.time() >= action.timeout_s) {
            advance();
            return;
        }

        action.waylist.updateBlended(drive.getPose());
        // Pathing style controls how targets are followed, not how accurately an action ends.
        // In particular, a continuous action must not hand off before it reaches its final pose.
        if (!action.waylist.isCompleteWithHeading(drive.getPose(),
                COMPLETION_HEADING_TOLERANCE_rad)) {
            completionStart_s = NO_COMPLETION_START_s;
            commandDrive(action);
        } else if (completionStart_s < 0.0) {
            completionStart_s = commandTimer.time();
        } else if (commandTimer.time() - completionStart_s >= action.minimumDwell_s) {
            advance();
        }
    }

    private void commandDrive(Action action) {
        Pose2d target = action.waylist.getBlendedTarget(drive.getPose(), LOOKAHEAD_m);
        drive.setTargetPose(target, action.maxVelocity_mps, action.translation_kP, action.rotation_kP,
                action.pathingStyle == ActionList.PathingStyle.CONTINUOUS);
        Logger.recordOutput("Auto/Target", target);
        Logger.recordOutput("Auto/PathProgress", action.waylist.getPathProgress());
    }

    private void advance() {
        actions.pass();
        startAction();
    }

    private void applySuperstructureFlags(Action action) {
        for (Flag flag : Flag.values()) superstructure.set(flag, action.flags.contains(flag));
    }

    private void clearSuperstructureFlags() {
        for (Flag flag : Flag.values()) superstructure.disable(flag);
    }

    @Override
    protected void outputPeriodic() {
        Logger.recordOutput("Auto/Name", actions.getName());
        Logger.recordOutput("Auto/ActionIndex", actions.getIndex());
        Logger.recordOutput("Auto/AllianceRed", Constants.isRedAlliance());
    }
}
