// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.devices.gyro.Gyro;
import frc.robot.devices.swervemodule.SwerveModule;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SubsystemBase;
import frc.robot.subsystems.obstacles.ObstacleAvoidance;
import frc.robot.subsystems.tracking.Tracking;
import frc.robot.util.ChassisAcceleration;
import frc.robot.util.Util;

public class Drive extends SubsystemBase<PathingMode> {
    // TunerConstants doesn't include these constants, so they are declared locally

    private static Drive instance;
    private static final int MODULE_COUNT = 4;
    private static final int FRONT_LEFT_INDEX = 0;
    private static final int FRONT_RIGHT_INDEX = 1;
    private static final int BACK_LEFT_INDEX = 2;
    private static final int BACK_RIGHT_INDEX = 3;
    private static final int ONE_MOTOR_PER_MODULE = 1;
    private static final double DEFAULT_FOLLOWER_MAX_SPEED_mps = 2.5;
    private static final double SIM_ANTISKEW_TIME_s = -0.0095;
    private static final double REAL_ANTISKEW_TIME_s = -0.002;
    private static final double FULL_TURN_deg = 180.0;

    // Constraints
    public static final double ODOMETRY_FREQUENCY_Hz =
            new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
    // SwerveModuleConstants stores LocationX/Y in meters (withLocationX does Distance.in(Meters)),
    // so these are already metric -- do not convert.
    public static final double DRIVE_BASE_RADIUS_m = Math.max(Math.max(
            Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX,
                            TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX,
                            TunerConstants.BackRight.LocationY)));

    public static final double MAX_VOLTAGE_V = 12.0;
    public static final double MAX_LINEAR_VEL_mps = 4.8; // TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                                                         // //
                                                         // 4.483;
    public static final double MAX_LINEAR_VEL_CONTROLLED_mps = 4.8; // 4
    public static final double MAX_LINEAR_VEL_THROTTLED_mps = 3.5;
    public static final double MAX_ANGULAR_VEL_radps = (MAX_LINEAR_VEL_mps / DRIVE_BASE_RADIUS_m);
    public static final double MAX_ANGULAR_VEL_THROTTLED_radps = MAX_ANGULAR_VEL_radps * 0.5;
    public static final double MAX_FORWARD_ACC_mps2 = 55.0;
    public static final double MAX_AUTO_FORWARD_ACC_mps2 = 55.0;
    public static final double MAX_ANGULAR_ACC_radps2 = 22.0 * MAX_ANGULAR_VEL_radps;
    public static final double MAX_TILT_XPOS_ACC_mps2 = 100, MAX_TILT_XNEG_ACC_mps2 = 100;
    public static final double MAX_TILT_YPOS_ACC_mps2 = 100, MAX_TILT_YNEG_ACC_mps2 = 100;
    public static final double MIN_TILT_XPOS_ACC_mps2 = 20, MIN_TILT_XNEG_ACC_mps2 = 20;
    public static final double MIN_TILT_YPOS_ACC_mps2 = 20, MIN_TILT_YNEG_ACC_mps2 = 20;
    public static final double MAX_SKID_ACC_mps2 = 90;

    static final Lock odometryLock = new ReentrantLock();

    private static final double ROBOT_MASS_KG = 74.088;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    
    public static RobotConfig PP_CONFIG = new RobotConfig(ROBOT_MASS_KG, ROBOT_MOI,
            new ModuleConfig(TunerConstants.FrontLeft.WheelRadius,
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), WHEEL_COF,
                    DCMotor.getKrakenX60Foc(ONE_MOTOR_PER_MODULE)
                            .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                    TunerConstants.FrontLeft.SlipCurrent, ONE_MOTOR_PER_MODULE),
            getModuleTranslations());

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    private final SwerveInput si;
    private ChassisSpeeds inputSpeeds;
    private ChassisSpeeds outputSpeeds;
    private ChassisSpeeds acc;
    private ChassisSpeeds lastSpeeds;
    private ChassisSpeeds measuredAcc;
    private boolean trackingRequested;
    private ChassisSpeeds requestedSpeeds;

    private PoseFollower poseFollower = new PoseFollower(new Pose2d(), DEFAULT_FOLLOWER_MAX_SPEED_mps);
    // Separate pose follower just for the tracking override's "drive to the tag" phase.
    private final PoseFollower trackingPoseFollower = new PoseFollower(new Pose2d(), DEFAULT_FOLLOWER_MAX_SPEED_mps);

    private Pose2d targetPose = new Pose2d();

    private final Gyro gyro = new Gyro("Drive/Gyro");
    private final SwerveModule[] modules = new SwerveModule[MODULE_COUNT]; // FL, FR, BL, BR
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {new SwerveModulePosition(), new SwerveModulePosition(),
                    new SwerveModulePosition(), new SwerveModulePosition()};
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics,
            rawGyroRotation, lastModulePositions, new Pose2d());

    private Drive() {
        super("Drive");
        modules[FRONT_LEFT_INDEX] = new SwerveModule(FRONT_LEFT_INDEX, TunerConstants.FrontLeft);
        modules[FRONT_RIGHT_INDEX] = new SwerveModule(FRONT_RIGHT_INDEX, TunerConstants.FrontRight);
        modules[BACK_LEFT_INDEX] = new SwerveModule(BACK_LEFT_INDEX, TunerConstants.BackLeft);
        modules[BACK_RIGHT_INDEX] = new SwerveModule(BACK_RIGHT_INDEX, TunerConstants.BackRight);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive,
                tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        System.out.println("STARTING THREAD");
        PhoenixOdometryThread.getInstance().start();

        si = new SwerveInput(SwerveInput.ZERO);
        inputSpeeds = new ChassisSpeeds();
        outputSpeeds = new ChassisSpeeds();
        acc = new ChassisSpeeds();
        lastSpeeds = new ChassisSpeeds();
        measuredAcc = new ChassisSpeeds();
        trackingRequested = false;
        gyro.zero(0.0);
        setCommand(PathingMode.DISABLED);
    }

    @Override
    public void inputPeriodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyro.readInputs();
        for (var module : modules) {
            module.updateInputs();
        }
        odometryLock.unlock();

        for (var module : modules) {
            module.inputPeriodic();
        }

        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled
                                                                        // together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[MODULE_COUNT];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[MODULE_COUNT];
            for (int moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyro.isConnected()) {
                // Use the real gyro angle
                rawGyroRotation = gyro.getOdometryYawPositions()[i];
            } else {
                // Derive heading from wheel deltas (bot-2026 parity). No unit hack:
                // WheelRadius is already stored in meters, so the twist is correct as-is.
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyro.isConnected() && Constants.currentMode != Mode.SIM);
    }

    @Override
    protected void handle() {
        switch (getCommand()) {
            case DISABLED:
                inputSpeeds = new ChassisSpeeds();
                break;
            case FIELD_RELATIVE:
                // System.out.println("field relative");
                double maxLinearVel_mps =
                        Util.lerp(MAX_LINEAR_VEL_THROTTLED_mps, MAX_LINEAR_VEL_mps, si.throttle);
                double maxAngularVel_radps = Util.lerp(MAX_ANGULAR_VEL_THROTTLED_radps,
                        MAX_ANGULAR_VEL_radps, si.throttle);

                // Circular input processing
                double inputMagnitude = Util.sqInput(Math.hypot(si.xi, si.yi));

                double x_ = si.xi * maxLinearVel_mps;
                double y_ = si.yi * maxLinearVel_mps;
                double w_ = si.wi * maxAngularVel_radps;

                if (inputMagnitude > 1.0) {
                    x_ = x_ / inputMagnitude;
                    y_ = y_ / inputMagnitude;
                }

                inputSpeeds = getAllianceFieldRelativeSpeeds(x_, y_, w_);
                

                if (trackingRequested) {
                    inputSpeeds = applyTrackingOverride(inputSpeeds);
                }

                requestedSpeeds = inputSpeeds;

                break;
            case POSE_FOLLOWING:
                // Apply the pose follower output to the speeds consumed by the acceleration
                // limiters and module output stage. Keeping this as a local variable causes the
                // robot to remain stationary even though the follower is producing a target.
                inputSpeeds = runPoseFollowing();
                requestedSpeeds = inputSpeeds;
                break;
        }

        ChassisSpeeds measuredSpeeds = getChassisSpeeds();
        measuredAcc =
                ChassisAcceleration.calculate(lastSpeeds, measuredSpeeds, Constants.globalDelta_s);
        lastSpeeds = measuredSpeeds;
        ChassisSpeeds inputAcc = ChassisAcceleration.fromChassisSpeeds(measuredSpeeds, inputSpeeds,
                Constants.globalDelta_s);

        acc = inputAcc;

        acc = accLimitForward(acc, measuredSpeeds);
        acc = accLimitAngular(acc, measuredSpeeds);
        acc = accLimitTilt(acc);
        acc = accLimitSkid(acc);

        outputSpeeds =
                ChassisAcceleration.fromAcceleration(measuredSpeeds, acc, Constants.globalDelta_s);

        // Antiskew
        double px = -outputSpeeds.vyMetersPerSecond;
        double py = outputSpeeds.vxMetersPerSecond;

        // Pre-rotates the velocity command by (omega * kSkew) radians to cancel steer lag, so
        // kSkew is effectively "how many seconds of azimuth lag am I cancelling".
        //
        // Sim and real are different plants and need different values. SwerveModuleIOSim mirrors
        // bot-2026's near-massless, ungeared sim steer, whose only remaining lag is the ~1 loop
        // between commanding an angle and the sim PID acting on it; -0.0095 is bot-2026's fitted
        // value for exactly that plant. The two go together -- change one and the robot slants
        // sideways when translating and rotating at once.
        //
        // -0.002 is this robot's existing real value and is untouched/unverified; the real geared
        // steer has more lag than sim, so this likely wants tuning ON the robot, not in sim.
        double kSkew = RobotBase.isSimulation() ? SIM_ANTISKEW_TIME_s : REAL_ANTISKEW_TIME_s;

        outputSpeeds.vxMetersPerSecond += px * outputSpeeds.omegaRadiansPerSecond * kSkew;
        outputSpeeds.vyMetersPerSecond += py * outputSpeeds.omegaRadiansPerSecond * kSkew;

        // Last thing before we command the modules: let obstacle avoidance cap the speed so the robot
        // can always brake before a wall. It only ever slows us down.
        outputSpeeds = ObstacleAvoidance.getInstance().constrain(getPose(), outputSpeeds);

        // Log speeds and accelerations
        Logger.recordOutput("Drive/Speeds/Input", new ChassisSpeeds(si.xi, si.yi, si.wi));
        Logger.recordOutput("Drive/Speeds/InputVel", inputSpeeds);
        Logger.recordOutput("Drive/Speeds/MeasuredVel", measuredSpeeds);
        Logger.recordOutput("Drive/Speeds/OutputVel", outputSpeeds);
        Logger.recordOutput("Drive/Speeds/InputAcc", inputAcc);
        Logger.recordOutput("Drive/Speeds/MeasuredAcc", measuredAcc);
        Logger.recordOutput("Drive/Speeds/OutputAcc", acc);
    }

    public boolean inBounds(double tz, double tx) {
        double k = 1;
        return tx < tz * k && tz > -tz * k;
    }

    private ChassisSpeeds runPoseFollowing() {
        Pose2d currentPose = getPose();
        ChassisSpeeds speeds = poseFollower.process(currentPose);
        return speeds;
    }

    @Override
    public void outputPeriodic() {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = outputSpeeds; // ChassisSpeeds.discretize(outputSpeeds,
                                                     // Constants.globalDelta_s);
        SwerveModuleState[] setpointStates =
                kinematics.toSwerveModuleStates(discreteSpeeds/* , CENTER_OF_ROT */);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_VEL_mps);

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[MODULE_COUNT];
        double maxModuleVel = 0.0;
        for (int i = 0; i < MODULE_COUNT; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
            if (Math.abs(
                    optimizedSetpointStates[i].speedMetersPerSecond) > MAX_LINEAR_VEL_CONTROLLED_mps) {
                maxModuleVel = Math.abs(optimizedSetpointStates[i].speedMetersPerSecond);
            }
        }

        // Determine if openloop control should be used
        SwerveModule.Mode mode = (maxModuleVel > MAX_LINEAR_VEL_CONTROLLED_mps) ? SwerveModule.Mode.HIGH_SPEED
                : SwerveModule.Mode.HIGH_CONTROL;

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        } else {
            for (var module : modules) {
                module.outputPeriodic(mode);
            }
        }

        // Log setpoint states
        Logger.recordOutput("Drive/Modules/Mode", mode);
        Logger.recordOutput("Drive/Modules/Setpoints", setpointStates);
        Logger.recordOutput("Drive/Modules/SetpointsOptimized", optimizedSetpointStates);
        Logger.recordOutput("Drive/TrackingRequested", trackingRequested);
        Logger.recordOutput("Drive/TargetPose", targetPose);
        Logger.recordOutput("Drive/GyroAngle", getRotation().getDegrees());
    }

    public boolean velUnder(double mag) {
        double measured = Math.hypot(getChassisSpeeds().vxMetersPerSecond,
                getChassisSpeeds().vyMetersPerSecond);

        return measured < mag;
    }

    /** Sets the driver-owned request for AprilTag alignment assistance. */
    public void setTrackingRequested(boolean requested) {
        trackingRequested = requested;
    }

    public void setInput(SwerveInput i) {
        si.set(i);
    }

    public void zeroGyro() {
        // Keep the estimator and field-relative controls aligned with the newly zeroed gyro.
        // The overload also applies the red-alliance heading convention used by auto resets.
        zeroGyro(0.0);
    }

    public ChassisSpeeds accLimitForward(ChassisSpeeds acc, ChassisSpeeds vel) {
        ChassisSpeeds res = acc;
        double accMag_mps2 = ChassisAcceleration.magnitude(acc);
        double velMag_mps = ChassisAcceleration.magnitude(vel);
        double accAng_rad = ChassisAcceleration.angle(acc);
        double velAng_rad = ChassisAcceleration.angle(vel);

        if (accMag_mps2 == 0.0) {
            return res;
        }

        if (velMag_mps == 0.0) {
            res.vxMetersPerSecond *= MAX_FORWARD_ACC_mps2 / accMag_mps2;
            res.vyMetersPerSecond *= MAX_FORWARD_ACC_mps2 / accMag_mps2;
        } else {

            double alpha = Math.cos(accAng_rad - velAng_rad);

            if (alpha > 0) {
                double max = DriverStation.isAutonomous() ? MAX_AUTO_FORWARD_ACC_mps2
                        : MAX_FORWARD_ACC_mps2;
                double maxFwdAcc = max * (1.0 - velMag_mps / MAX_LINEAR_VEL_mps);
                double outMag = Math.min(accMag_mps2, maxFwdAcc);
                res.vxMetersPerSecond *= outMag / accMag_mps2;
                res.vyMetersPerSecond *= outMag / accMag_mps2;
            }
        }

        return res;

    }

    public ChassisSpeeds accLimitAngular(ChassisSpeeds acc, ChassisSpeeds vel) {
        ChassisSpeeds res = acc;

        double maxAngAcc = MAX_ANGULAR_ACC_radps2
                * (1.0 - Math.abs(vel.omegaRadiansPerSecond / MAX_ANGULAR_VEL_radps));

        double angularAcc_radps2 =
                Math.copySign(Math.min(maxAngAcc, Math.abs(acc.omegaRadiansPerSecond)),
                        acc.omegaRadiansPerSecond);
        res.omegaRadiansPerSecond = angularAcc_radps2;

        return res;
    }

    public ChassisSpeeds accLimitTilt(ChassisSpeeds in) {
        ChassisSpeeds res = in;
        // double h = (1.0 - target_bias) * Elevator.getInstance().getHeight()
        // + target_bias * Elevator.getInstance().getTargetHeight();

        double alpha = 1.0; // Util.unlerp(Elevator.MAX_HEIGHT_m, Elevator.MIN_HEIGHT_m, h);

        double maxXPosAcc = Util.lerp(MIN_TILT_XPOS_ACC_mps2, MAX_TILT_XPOS_ACC_mps2, alpha);
        double maxXNegAcc = Util.lerp(MIN_TILT_XNEG_ACC_mps2, MAX_TILT_XNEG_ACC_mps2, alpha);
        double maxYPosAcc = Util.lerp(MIN_TILT_YPOS_ACC_mps2, MAX_TILT_YPOS_ACC_mps2, alpha);
        double maxYNegAcc = Util.lerp(MIN_TILT_YNEG_ACC_mps2, MAX_TILT_YNEG_ACC_mps2, alpha);

        res.vxMetersPerSecond = Util.limit(in.vxMetersPerSecond, -maxXNegAcc, maxXPosAcc);
        res.vyMetersPerSecond = Util.limit(in.vyMetersPerSecond, -maxYNegAcc, maxYPosAcc);

        return res;
    }

    public ChassisSpeeds accLimitSkid(ChassisSpeeds acc) {
        ChassisSpeeds res = acc;
        double accMag_mps2 = ChassisAcceleration.magnitude(acc);

        double outMag = Math.min(accMag_mps2, MAX_SKID_ACC_mps2);
        if (accMag_mps2 != 0.0) {
            res = acc.times(outMag / accMag_mps2);
        }

        return res;
    }

    /** Stops the drive. */
    public void stop() {
        setInput(new SwerveInput(0, 0, 0));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[MODULE_COUNT];
        for (int i = 0; i < MODULE_COUNT; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[MODULE_COUNT];
        for (int i = 0; i < MODULE_COUNT; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the driver-requested chassis speeds. */
    public ChassisSpeeds getRequestedSpeeds() {
        return requestedSpeeds != null ? requestedSpeeds : new ChassisSpeeds();
    }

    public ChassisSpeeds getMeasuredChassisAcceleration() {
        return measuredAcc;
    }

    public ChassisSpeeds getNextChassisAcceleration() {
        return acc;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
        gyro.zero(pose.getRotation().getDegrees());
    }

    public void resetPose(Pose2d pose) {
        setPose(pose);
    }

    public void setTargetPose(Pose2d tPose, double maxVel, double translation_kP,
            double rotation_kP) {
        setTargetPose(tPose, maxVel, translation_kP, rotation_kP, false);
    }

    /** Commands a target pose, optionally preserving speed across autonomous waypoints. */
    public void setTargetPose(Pose2d tPose, double maxVel, double translation_kP,
            double rotation_kP, boolean preserveMomentum) {
        this.targetPose = tPose;
        if (!preserveMomentum) {
            this.poseFollower.reset();
        }
        this.poseFollower.setParams(tPose, maxVel, translation_kP, rotation_kP, preserveMomentum);
        setCommand(PathingMode.POSE_FOLLOWING);
    }

    public boolean isTrackingRequested() {
        return trackingRequested;
    }


    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds) {
        odometryLock.lock();
        poseEstimator.addVisionMeasurement(visionRobotPose, timestampSeconds);
        odometryLock.unlock();
    }

    public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        odometryLock.lock();
        poseEstimator.addVisionMeasurement(visionRobotPose, timestampSeconds,
                visionMeasurementStdDevs);
        odometryLock.unlock();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return this.poseEstimator;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
                new Translation2d(TunerConstants.FrontLeft.LocationX,
                        TunerConstants.FrontLeft.LocationY),
                new Translation2d(TunerConstants.FrontRight.LocationX,
                        TunerConstants.FrontRight.LocationY),
                new Translation2d(TunerConstants.BackLeft.LocationX,
                        TunerConstants.BackLeft.LocationY),
                new Translation2d(TunerConstants.BackRight.LocationX,
                        TunerConstants.BackRight.LocationY)};
    }

    private ChassisSpeeds getAllianceFieldRelativeSpeeds(double vx, double vy, double omega) {
        if (Constants.isRedAlliance()) {
            vx = -vx;
            vy = -vy;
        }
        return ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getRotation());
    }

    // Called while the tracking override is on. Given the driver's requested speeds, return what the
    // robot should actually do. Three cases:
    private ChassisSpeeds applyTrackingOverride(ChassisSpeeds driverSpeeds) {
        Tracking tracking = Tracking.getInstance();
        Logger.recordOutput("Drive/Tracking/TagId", tracking.getTargetTag());

        if (!tracking.isEnabled()) {
            return driverSpeeds;
        }

        // 1) Tag is in view -> run the final tracking (turn to it + drive in to the standoff).
        if (tracking.seesTarget()) {
            return tracking.getTargetSpeeds(getRotation().getDegrees(), MAX_LINEAR_VEL_THROTTLED_mps);
        }
        // 2) No tag chosen at all -> leave the driver in control.
        if (tracking.getTargetTag() == -1) {
            return driverSpeeds;
        }
        // 3) Chosen a tag but can't see it yet -> drive to its approach pose so it comes into view.
        //    Use the pose follower for x/y, but our own gentle rotation for heading (the follower's
        //    rotation is too aggressive and spins).
        trackingPoseFollower.setParams(tracking.getApproachPose(), MAX_LINEAR_VEL_THROTTLED_mps);
        ChassisSpeeds follow = trackingPoseFollower.process(getPose());
        double omega_radps = tracking.approachRotation(getRotation().getDegrees());
        return new ChassisSpeeds(follow.vxMetersPerSecond, follow.vyMetersPerSecond, omega_radps);
    }

    public void resetGyroAndPose(double gyroAngle, Pose2d currentPose) {
        clearMotionState();
        double adjustedAngle = adjustForAlliance(gyroAngle);

        gyro.zero(adjustedAngle);
        rawGyroRotation = Rotation2d.fromDegrees(adjustedAngle);

        Pose2d adjustedPose = new Pose2d(
            currentPose.getX(),
            currentPose.getY(),
            rawGyroRotation
        );

        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), adjustedPose);
    }

    /** Clears any in-flight chassis command before establishing a new autonomous pose. */
    private void clearMotionState() {
        inputSpeeds = new ChassisSpeeds();
        outputSpeeds = new ChassisSpeeds();
        acc = new ChassisSpeeds();
        lastSpeeds = new ChassisSpeeds();
        measuredAcc = new ChassisSpeeds();
        poseFollower.reset();
        setCommand(PathingMode.DISABLED);
    }

    public void zeroGyro(double deg) {
        double adjustedAngle = adjustForAlliance(deg);

        gyro.zero(adjustedAngle);
        rawGyroRotation = Rotation2d.fromDegrees(adjustedAngle);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), new Pose2d(getPose().getX(), getPose().getY(), rawGyroRotation));
    }

    private double adjustForAlliance(double angleDeg) {
        System.out.println(Constants.isRedAlliance());
        if (Constants.isRedAlliance()) {
            return (angleDeg + FULL_TURN_deg) % (2.0 * FULL_TURN_deg);
        }
        return angleDeg;
    }

    public static Pose2d flipPoseForRed(Pose2d bluePose) {
        if (Constants.isRedAlliance()) {
            double newX = Constants.FIELD_LENGTH_M - bluePose.getX();
            double newY = Constants.FIELD_WIDTH_M - bluePose.getY();
            Rotation2d newRotation = bluePose.getRotation().rotateBy(Rotation2d.fromDegrees(FULL_TURN_deg));
            return new Pose2d(newX, newY, newRotation);
        } else {
            return bluePose;
        }
    }
}
