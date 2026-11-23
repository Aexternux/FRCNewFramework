package frc.robot.application.auto;

import java.util.ArrayDeque;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.application.auto.ActionList.Action;
import frc.robot.application.superstructure.SS;
import frc.robot.application.superstructure.SS.Flag;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.vision.Vision;

/** Runs one {@link ActionList} using the drivetrain pose follower. */
public class Auto extends SubsystemBase<AutoState> {
    private static final double LOOKAHEAD_m = 1.2;
    private static final double FULL_TURN_deg = 180.0;
    private static final double NO_COMPLETION_START_s = -1.0;

    // Anti-beach: a leg commanding motion while the robot hasn't moved this long is stuck.
    private static final double STUCK_WINDOW_s = 1.5;
    private static final double STUCK_MIN_COMMAND_mps = 0.5;
    private static final double STUCK_MAX_MOVEMENT_m = 0.15;
    // Wheels spinning on a ball fool odometry, so vision only counts if it spans most of the window.
    private static final double STUCK_MIN_VISION_SPAN_s = 1.0;
    private static final int MAX_ESCAPES_PER_LEG = 2;
    private static final double ESCAPE_MOVE_s = 0.4;
    private static final double ESCAPE_SPEED_mps = 2.0;
    // Escape targets sit this far out so they're never reached inside one timed move.
    private static final double ESCAPE_TARGET_DISTANCE_m = 1.0;
    private static final double ESCAPE_TRANSLATION_kP = 3.0;
    private static final double ESCAPE_ROTATION_kP = 0.5;
    private static final double ESCAPE_WIGGLE_deg = 40.0;
    private static final double NO_TIME_s = -1.0;
    private static final int NO_ESCAPE = -1;

    /** Escape moves, tried in order. Directions are relative to the way the leg was heading. */
    private enum EscapeMove {
        BACK_OUT(180.0, 0.0),
        STRAFE_LEFT(90.0, 0.0),
        STRAFE_RIGHT(-90.0, 0.0),
        WIGGLE_LEFT(0.0, ESCAPE_WIGGLE_deg),
        WIGGLE_RIGHT(0.0, -ESCAPE_WIGGLE_deg);

        final double direction_deg;
        final double headingOffset_deg;

        EscapeMove(double direction_deg, double headingOffset_deg) {
            this.direction_deg = direction_deg;
            this.headingOffset_deg = headingOffset_deg;
        }
    }

    private record MotionSample(double t_s, Translation2d odometry_m, Translation2d vision_m) {}

    private final Drive drive = Drive.getInstance();
    private final SS superstructure = SS.getInstance();
    private final Vision vision = Vision.getInstance();
    private final ActionList actions;
    private double completionStart_s = NO_COMPLETION_START_s;

    private final ArrayDeque<MotionSample> motionHistory = new ArrayDeque<>();
    private double commandedSince_s = NO_TIME_s;
    private int escapeIndex = NO_ESCAPE;
    private int escapesThisLeg;
    private double escapeMoveStart_s;
    private Rotation2d escapeTravel = Rotation2d.kZero;
    private Rotation2d escapeHeading = Rotation2d.kZero;
    private boolean antiBeachEnabled = true;

    public Auto(ActionList actions) {
        super("Auto");
        this.actions = actions;
        setCommand(AutoState.IDLE);
    }

    public void setAntiBeachEnabled(boolean enabled) {
        antiBeachEnabled = enabled;
    }

    /** Resets path progress and odometry, then starts the first action. */
    public void start() {
        actions.reset();
        // Applying the 180-degree field transform twice restores blue coordinates. Tracking the
        // applied alliance on the shared ActionList keeps reruns correct no matter when the
        // DriverStation reports alliance.
        boolean redAlliance = Constants.isRedAlliance();
        if (redAlliance != actions.isFlippedToRed()) {
            actions.transformWaylists(pose -> new Pose2d(
                    Constants.FIELD_LENGTH_M - pose.getX(), Constants.FIELD_WIDTH_M - pose.getY(),
                    pose.getRotation().rotateBy(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(FULL_TURN_deg))));
            actions.setFlippedToRed(redAlliance);
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
        Logger.recordOutput(scope() + "/StartPose", startPose);
    }

    public void stop() {
        clearSuperstructureFlags();
        drive.setCommand(PathingMode.DISABLED);
        setCommand(AutoState.DISABLED);
    }

    @Override
    public void inputPeriodic() {}

    @Override
    public void handle() {
        if (DriverStation.isDisabled()) {
            stop();
            return;
        }
        switch (getCommand()) {
            case DRIVE:
                handleDrive();
                break;
            case WAIT: {
                Action waitAction = actions.currentAction();
                if (waitAction == null || commandTimer.time() >= waitAction.timeout_s) advance();
                break;
            }
            case FINISHED:
            case IDLE:
            case DISABLED:
                break;
        }
    }

    private void startAction() {
        Action action = actions.currentAction();
        completionStart_s = NO_COMPLETION_START_s;
        resetStuckTracking();
        escapeIndex = NO_ESCAPE;
        escapesThisLeg = 0;
        commandTimer.reset();
        if (action == null) {
            clearSuperstructureFlags();
            drive.setCommand(PathingMode.DISABLED);
            setCommand(AutoState.FINISHED);
            return;
        }
        clearSuperstructureFlags();
        applySuperstructureFlags(action);
        if (action.type == ActionList.ActionType.DRIVE) {
            setCommand(AutoState.DRIVE);
            commandDrive(action);
        } else {
            if (actions.getIndex() == 0) {
                drive.setCommand(PathingMode.DISABLED);
            }
            setCommand(AutoState.WAIT);
        }
    }

    private void handleDrive() {
        Action action = actions.currentAction();
        if (action == null || commandTimer.time() >= action.timeout_s) {
            advance();
            return;
        }
        if (escapeIndex != NO_ESCAPE) {
            runEscape(action);
            return;
        }

        action.waylist.updateBlended(drive.getPose());
        if (!action.waylist.isComplete(drive.getPose())) {
            completionStart_s = NO_COMPLETION_START_s;
            if (antiBeachEnabled && isStuck() && escapesThisLeg < MAX_ESCAPES_PER_LEG) {
                startEscape(action);
                runEscape(action);
            } else {
                commandDrive(action);
            }
            return;
        }
        Action next = actions.peekNext();
        if (next != null && next.type == ActionList.ActionType.DRIVE) {
            advance();
            return;
        }
        // Legs complete on position alone; heading during a following wait belongs to SS (e.g. AIM).
        if (completionStart_s < 0.0) {
            completionStart_s = commandTimer.time();
        } else if (commandTimer.time() - completionStart_s >= action.minimumDwell_s) {
            advance();
        }
    }

    private void commandDrive(Action action) {
        Pose2d target = action.waylist.getBlendedTarget(drive.getPose(), LOOKAHEAD_m);
        drive.setTargetPose(target, action.maxVelocity_mps, action.translation_kP, action.rotation_kP,
                action.pathingStyle == ActionList.PathingStyle.CONTINUOUS);
        Logger.recordOutput(scope() + "/Target", target);
        Logger.recordOutput(scope() + "/PathProgress", action.waylist.getPathProgress());
    }

    private void resetStuckTracking() {
        motionHistory.clear();
        commandedSince_s = NO_TIME_s;
    }

    /**
     * True once the leg has asked for motion for {@link #STUCK_WINDOW_s} but the robot hasn't moved.
     * Either odometry or vision seeing no movement counts; vision catches wheels spinning in place.
     */
    private boolean isStuck() {
        double now_s = commandTimer.time();
        ChassisSpeeds requested = drive.getRequestedSpeeds();
        boolean commanding = Math.hypot(requested.vxMetersPerSecond, requested.vyMetersPerSecond)
                >= STUCK_MIN_COMMAND_mps;
        if (!commanding) {
            resetStuckTracking();
            return false;
        }
        if (commandedSince_s < 0.0) {
            commandedSince_s = now_s;
        }
        Translation2d visionNow_m = vision.hasFusedPose() ? vision.getFusedPose().getTranslation() : null;
        motionHistory.addLast(new MotionSample(now_s, drive.getPose().getTranslation(), visionNow_m));
        while (now_s - motionHistory.peekFirst().t_s() > STUCK_WINDOW_s) {
            motionHistory.removeFirst();
        }
        double commanded_s = now_s - commandedSince_s;
        if (commanded_s < STUCK_WINDOW_s) {
            return false;
        }

        double odometryMoved_m =
                motionHistory.peekLast().odometry_m().getDistance(motionHistory.peekFirst().odometry_m());
        MotionSample firstVision = null;
        MotionSample lastVision = null;
        for (MotionSample sample : motionHistory) {
            if (sample.vision_m() != null) {
                if (firstVision == null) {
                    firstVision = sample;
                }
                lastVision = sample;
            }
        }
        boolean visionValid = firstVision != null
                && lastVision.t_s() - firstVision.t_s() >= STUCK_MIN_VISION_SPAN_s;
        double visionMoved_m = visionValid
                ? lastVision.vision_m().getDistance(firstVision.vision_m()) : Double.NaN;

        Logger.recordOutput(scope() + "/Stuck/OdometryMoved_m", odometryMoved_m);
        Logger.recordOutput(scope() + "/Stuck/VisionMoved_m", visionMoved_m);
        return odometryMoved_m < STUCK_MAX_MOVEMENT_m
                || (visionValid && visionMoved_m < STUCK_MAX_MOVEMENT_m);
    }

    private void startEscape(Action action) {
        Pose2d pose = drive.getPose();
        Translation2d toTarget =
                action.waylist.getBlendedTarget(pose, LOOKAHEAD_m).getTranslation().minus(pose.getTranslation());
        escapeTravel = toTarget.getNorm() > 1e-3 ? toTarget.getAngle() : pose.getRotation();
        escapeHeading = pose.getRotation();
        escapeIndex = 0;
        escapeMoveStart_s = commandTimer.time();
        escapesThisLeg++;
        resetStuckTracking();
        Logger.recordOutput(scope() + "/Stuck/EscapeCount", escapesThisLeg);
        Logger.recordOutput(scope() + "/Stuck/StuckPose", pose);
    }

    /** Runs one loop of the timed escape sequence, handing back to the leg once it's done. */
    private void runEscape(Action action) {
        double now_s = commandTimer.time();
        if (now_s - escapeMoveStart_s >= ESCAPE_MOVE_s) {
            escapeIndex++;
            escapeMoveStart_s = now_s;
        }
        if (escapeIndex >= EscapeMove.values().length) {
            escapeIndex = NO_ESCAPE;
            Logger.recordOutput(scope() + "/Stuck/EscapeMove", "NONE");
            action.waylist.updateBlended(drive.getPose());
            commandDrive(action);
            return;
        }
        EscapeMove move = EscapeMove.values()[escapeIndex];
        Pose2d pose = drive.getPose();
        Rotation2d direction = escapeTravel.plus(Rotation2d.fromDegrees(move.direction_deg));
        // Re-anchored on the current pose every loop, so slipping odometry can't "arrive" early.
        Pose2d target = new Pose2d(
                pose.getTranslation().plus(new Translation2d(ESCAPE_TARGET_DISTANCE_m, direction)),
                escapeHeading.plus(Rotation2d.fromDegrees(move.headingOffset_deg)));
        drive.setTargetPose(target, ESCAPE_SPEED_mps, ESCAPE_TRANSLATION_kP, ESCAPE_ROTATION_kP, false);
        Logger.recordOutput(scope() + "/Stuck/EscapeMove", move.name());
        Logger.recordOutput(scope() + "/Target", target);
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
    public void outputPeriodic() {
        Logger.recordOutput(scope() + "/Name", actions.getName());
        Logger.recordOutput(scope() + "/ActionIndex", actions.getIndex());
        Logger.recordOutput(scope() + "/AllianceRed", Constants.isRedAlliance());
        Logger.recordOutput(scope() + "/AntiBeachEnabled", antiBeachEnabled);
    }
}
