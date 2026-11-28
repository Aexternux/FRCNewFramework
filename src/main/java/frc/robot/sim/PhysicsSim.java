package frc.robot.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.Constants;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

/**
 * Connects the robot code's simulated Phoenix devices to the CAD physics simulator (CadParser).
 *
 * <p>Robot code runs unchanged: the same TalonFX objects, configs and control requests as on the
 * robot, and Phoenix 6 simulates the Talon firmware (Motion Magic, PID, current limits). What the
 * firmware outputs goes to the physics instead of a motor; what the physics measures comes back
 * as the motor's rotor, the CANcoders and the Pigeon. The mechanism, its gears, gravity, game
 * pieces and contacts are all the physics':
 *
 * <pre>
 *   robot code -> TalonFX (Phoenix firmware sim) -> volts --NT--> physics (CAD robot)
 *   robot code <- TalonFX / CANcoder / Pigeon2  <- rotor, angles <--NT-- physics
 * </pre>
 *
 * <p>Everything on NetworkTables is in the code's MECHANISM sense: positive is what the code calls
 * positive (after the configured invert), so the physics never needs to know how a motor is wired.
 * Topics under /PhysicsSim:
 *
 * <ul>
 *   <li>devices (string, JSON): every registered device, published by this side
 *   <li>motor/KEY/volts, motor/KEY/brake: what the firmware applies (robot to physics)
 *   <li>motor/KEY/rotor: [position_rot, velocity_rps] of the motor's rotor (physics to robot)
 *   <li>cancoder/KEY: [absolute_rot, velocity_rps], gyro: [yaw_deg, yawRate_dps, placeSeq, pitch_deg, roll_deg], truthPose:
 *       [x_m, y_m, yaw_rad] (physics to robot)
 *   <li>place: [x_m, y_m, yaw_rad, seq] -- where the code says the robot is (odometry reset at start,
 *       an auto's start pose): the simulator puts the physics robot there, as a person places the
 *       robot before a match (robot to physics)
 *   <li>ready: a counter the simulator page bumps on "Robot ready" -- every motor's position is set to 0
 *   <li>dsMode: "disabled" / "teleop" / "auto" from the simulator page, applied to the simulated
 *       Driver Station; robotState: what the robot is doing (robot to physics)
 * </ul>
 */
public final class PhysicsSim {

    private static final String ROOT = "/PhysicsSim";
    /** Device bridge rate: faster than the robot loop, like the CAN status frames it stands in for. */
    private static final double UPDATE_PERIOD_s = 0.004;
    /** Between physics updates a reading is advanced by its velocity, at most this long. */
    private static final double MAX_EXTRAPOLATION_s = 0.05;
    /** A physics update older than this means the simulator isn't connected. */
    private static final double STALE_s = 0.5;

    private static PhysicsSim instance;
    private static final long BOOT_MS = System.currentTimeMillis();

    /** True when this run should use the physics simulator (simulation + the Constants switch). */
    public static boolean isEnabled() {
        return Constants.currentMode == Constants.Mode.SIM && Constants.physicsSim;
    }

    public static synchronized PhysicsSim getInstance() {
        if (instance == null) {
            instance = new PhysicsSim();
        }
        return instance;
    }

    /** A motor (leader plus followers) the physics drives. */
    private static final class SimMotor {
        final String key;
        final String name;
        final String role;
        final int module;
        final TalonFX leader;
        final List<TalonFX> followers;
        final List<Boolean> followerOpposed;
        final double mechanismSign;
        /** The code's SensorToMechanismRatio (rotor turns per mechanism turn), so the simulator can check it. */
        double ratio = 0.0;
        volatile boolean brake;
        DoublePublisher voltsPub;
        BooleanPublisher brakePub;
        DoubleArraySubscriber rotorSub;

        SimMotor(String key, String name, String role, int module, TalonFX leader, List<TalonFX> followers,
                List<Boolean> followerOpposed, boolean inverted, boolean brake) {
            this.key = key;
            this.name = name;
            this.role = role;
            this.module = module;
            this.leader = leader;
            this.followers = followers;
            this.followerOpposed = followerOpposed;
            this.mechanismSign = inverted ? -1.0 : 1.0;
            this.brake = brake;
        }
    }

    /** A CANcoder: the physics reports the angle the code should read; this side undoes the offset. */
    private static final class SimEncoder {
        final String key;
        final CANcoder cancoder;
        final double offset_rot;
        final double direction;
        DoubleArraySubscriber sub;

        SimEncoder(String key, CANcoder cancoder, double offset_rot, boolean clockwisePositive) {
            this.key = key;
            this.cancoder = cancoder;
            this.offset_rot = offset_rot;
            this.direction = clockwisePositive ? -1.0 : 1.0;
        }
    }

    /** A swerve module's devices and geometry, so the physics can match it to a CAD module. */
    private record SimModule(int index, String drive, String steer, String encoder, double x_m, double y_m,
            double driveRatio, double steerRatio, double wheelRadius_m, double couplingRatio) {}

    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private final NetworkTable table = nt.getTable(ROOT);
    private final StringPublisher devicesPub = table.getStringTopic("devices").publish();
    private final DoubleArraySubscriber gyroSub;
    private final DoubleArraySubscriber truthPoseSub;
    private final DoubleArrayPublisher placePub = table.getDoubleArrayTopic("place").publish();
    private final StringSubscriber dsModeSub = table.getStringTopic("dsMode").subscribe("");
    private final StringPublisher robotStatePub = table.getStringTopic("robotState").publish();
    private final edu.wpi.first.networktables.DoubleSubscriber readySub = table.getDoubleTopic("ready").subscribe(0.0);
    private double readySeen = 0.0;
    /** Last time (FPGA s) the robot was disabled: an auto's start pose comes right after. */
    private double lastDisabled_s = 0.0;
    private String dsMode = "";
    /** Placements sent; gyro readings older than the last one are from before the robot was moved. */
    private volatile int placeSeq = 0;

    private final List<SimMotor> motors = new ArrayList<>();
    private final List<SimEncoder> encoders = new ArrayList<>();
    private final List<SimModule> modules = new ArrayList<>();
    private Pigeon2 pigeon;
    private String pigeonKey;

    private final Notifier notifier = new Notifier(this::update);
    private boolean started = false;
    private volatile boolean connected = false;

    private PhysicsSim() {
        gyroSub = table.getDoubleArrayTopic("gyro").subscribe(new double[] {}, fast());
        truthPoseSub = table.getDoubleArrayTopic("truthPose").subscribe(new double[] {}, fast());
        notifier.setName("PhysicsSim");
    }

    private static PubSubOption[] fast() {
        return new PubSubOption[] {PubSubOption.periodic(0.005), PubSubOption.keepDuplicates(true)};
    }

    private static String key(String canbus, int canId) {
        return (canbus == null || canbus.isEmpty() || canbus.equals("rio")) ? Integer.toString(canId)
                : canbus + "-" + canId;
    }

    /**
     * Registers a motor and its followers with the physics.
     *
     * @param name what the simulator shows it as (subsystem/motor)
     * @param role "motor", or "drive" / "steer" for a swerve module's motors
     * @param module the swerve module index, or -1
     */
    public synchronized void registerMotor(String name, String role, int module, String canbus, TalonFX leader,
            List<TalonFX> followers, List<Boolean> followerOpposed, boolean inverted, boolean brake) {
        registerMotor(name, role, module, canbus, leader, followers, followerOpposed, inverted, brake, 0.0);
    }

    /** As above, with the code's SensorToMechanismRatio (0 if not known). */
    public synchronized void registerMotor(String name, String role, int module, String canbus, TalonFX leader,
            List<TalonFX> followers, List<Boolean> followerOpposed, boolean inverted, boolean brake, double ratio) {
        SimMotor m = new SimMotor(key(canbus, leader.getDeviceID()), name, role, module, leader, followers,
                followerOpposed, inverted, brake);
        m.ratio = ratio;
        NetworkTable t = table.getSubTable("motor").getSubTable(m.key);
        m.voltsPub = t.getDoubleTopic("volts").publish(fast());
        m.brakePub = t.getBooleanTopic("brake").publish(fast());
        m.rotorSub = t.getDoubleArrayTopic("rotor").subscribe(new double[] {}, fast());
        m.brakePub.set(brake);
        motors.add(m);
        changed();
    }

    /** Registers a CANcoder (its configured magnet offset and direction, so readings match the robot). */
    public synchronized void registerEncoder(String canbus, CANcoder cancoder, double offset_rot,
            boolean clockwisePositive) {
        SimEncoder e = new SimEncoder(key(canbus, cancoder.getDeviceID()), cancoder, offset_rot, clockwisePositive);
        e.sub = table.getSubTable("cancoder").getDoubleArrayTopic(e.key).subscribe(new double[] {}, fast());
        encoders.add(e);
        changed();
    }

    /** Registers a swerve module (its devices must be registered too). Location in the robot frame. */
    public synchronized void registerModule(int index, String canbus, int driveId, int steerId, int encoderId,
            double x_m, double y_m, double driveRatio, double steerRatio, double wheelRadius_m,
            double couplingRatio) {
        modules.add(new SimModule(index, key(canbus, driveId), key(canbus, steerId), key(canbus, encoderId),
                x_m, y_m, driveRatio, steerRatio, wheelRadius_m, couplingRatio));
        changed();
    }

    public synchronized void registerGyro(String canbus, Pigeon2 pigeon) {
        this.pigeon = pigeon;
        this.pigeonKey = key(canbus, pigeon.getDeviceID());
        changed();
    }

    /** The motor's neutral mode changed (brake shorts the windings; coast lets the mechanism spin). */
    public synchronized void setBrake(TalonFX leader, boolean brake) {
        for (SimMotor m : motors) {
            if (m.leader == leader) {
                m.brake = brake;
                m.brakePub.set(brake);
            }
        }
    }

    private void changed() {
        devicesPub.set(devicesJson());
        if (!started) {
            started = true;
            notifier.startPeriodic(UPDATE_PERIOD_s);
        }
    }

    private String devicesJson() {
        // (boot: when this program started -- the simulator puts the robot in its start configuration then)
        StringBuilder s = new StringBuilder("{\"version\":1,\"boot\":" + BOOT_MS + ",\"motors\":[");
        for (int i = 0; i < motors.size(); i++) {
            SimMotor m = motors.get(i);
            s.append(i == 0 ? "" : ",").append("{\"key\":\"").append(m.key).append("\",\"name\":\"")
                    .append(m.name.replace("\"", "'")).append("\",\"canId\":").append(m.leader.getDeviceID())
                    .append(",\"count\":").append(1 + m.followers.size()).append(",\"role\":\"").append(m.role)
                    .append("\",\"module\":").append(m.module).append(",\"ratio\":").append(m.ratio).append("}");
        }
        s.append("],\"modules\":[");
        for (int i = 0; i < modules.size(); i++) {
            SimModule m = modules.get(i);
            s.append(i == 0 ? "" : ",").append("{\"index\":").append(m.index()).append(",\"drive\":\"")
                    .append(m.drive()).append("\",\"steer\":\"").append(m.steer()).append("\",\"encoder\":\"")
                    .append(m.encoder()).append("\",\"x\":").append(m.x_m()).append(",\"y\":").append(m.y_m())
                    .append(",\"driveRatio\":").append(m.driveRatio()).append(",\"steerRatio\":")
                    .append(m.steerRatio()).append(",\"wheelRadius\":").append(m.wheelRadius_m())
                    .append(",\"couplingRatio\":").append(m.couplingRatio()).append("}");
        }
        s.append("]");
        if (pigeonKey != null) {
            s.append(",\"gyro\":{\"key\":\"").append(pigeonKey).append("\"}");
        }
        return s.append("}").toString();
    }

    /** [position, velocity] advanced by its age; null before the physics has sent one. */
    private static double[] extrapolated(DoubleArraySubscriber sub, long now_us) {
        TimestampedDoubleArray v = sub.getAtomic();
        if (v.value.length < 2) {
            return null;
        }
        double age_s = (now_us - v.timestamp) / 1e6;
        // (an age outside 0..STALE means the simulator's clock isn't synced yet: don't guess)
        age_s = age_s < 0.0 || age_s > STALE_s ? 0.0 : Math.min(MAX_EXTRAPOLATION_s, age_s);
        return new double[] {v.value[0] + v.value[1] * age_s, v.value[1]};
    }

    /** Device bridge, on its own thread: firmware output to the physics, physics readings to the devices. */
    private synchronized void update() {
        long now_us = WPIUtilJNI.now();
        double battery_V = RobotController.getBatteryVoltage();
        if (!(battery_V > 4.0)) {
            battery_V = 12.0;      // (no simulated battery: HAL not running, e.g. a unit test)
        }
        long newest_us = 0;

        for (SimMotor m : motors) {
            TalonFXSimState leaderSim = m.leader.getSimState();
            leaderSim.setSupplyVoltage(battery_V);
            m.voltsPub.set(m.mechanismSign * leaderSim.getMotorVoltage());

            double[] r = extrapolated(m.rotorSub, now_us);
            if (r == null) {
                continue;
            }
            newest_us = Math.max(newest_us, m.rotorSub.getAtomic().timestamp);
            double rawPos_rot = m.mechanismSign * r[0];
            double rawVel_rps = m.mechanismSign * r[1];
            leaderSim.setRawRotorPosition(rawPos_rot);
            leaderSim.setRotorVelocity(rawVel_rps);
            for (int i = 0; i < m.followers.size(); i++) {
                TalonFXSimState f = m.followers.get(i).getSimState();
                double sign = m.followerOpposed.get(i) ? -1.0 : 1.0;
                f.setSupplyVoltage(battery_V);
                f.setRawRotorPosition(sign * rawPos_rot);
                f.setRotorVelocity(sign * rawVel_rps);
            }
        }

        for (SimEncoder e : encoders) {
            double[] r = extrapolated(e.sub, now_us);
            if (r == null) {
                continue;
            }
            // reported = direction * raw + offset (pinned down in PhoenixSimConventionsTest)
            e.cancoder.getSimState().setRawPosition(e.direction * (r[0] - e.offset_rot));
            e.cancoder.getSimState().setVelocity(e.direction * r[1]);
        }

        if (pigeon != null) {
            double[] g = extrapolated(gyroSub, now_us);
            double[] raw = gyroSub.get();
            boolean placed = raw.length < 3 || raw[2] >= placeSeq;     // (the simulator has moved the robot)
            if (g != null && placed) {
                pigeon.getSimState().setRawYaw(g[0]);
                pigeon.getSimState().setAngularVelocityZ(g[1]);
            }
            if (raw.length >= 5) {     // the robot tilting on the field's bumps (right-handed: +pitch = nose down)
                pigeon.getSimState().setPitch(raw[3]);
                pigeon.getSimState().setRoll(raw[4]);
            }
        }

        connected = newest_us > 0 && (now_us - newest_us) / 1e6 < STALE_s;
        nt.flush();
    }

    /**
     * The code has just set where the robot is (an odometry reset): put the physics robot there.
     * Call BEFORE zeroing the gyro: the simulated Pigeon is set to the new heading here, and readings
     * from the simulator are ignored until it confirms it has moved the robot.
     */
    public synchronized void placeRobot(Pose2d pose) {
        // Only where a person would put the robot: while it's disabled, or an auto setting its start pose
        // as it begins. A pose reset in the middle of driving (a driver's "manual zero" button) only tells
        // the code where it is -- the robot doesn't move. (Teleporting it there dropped it on balls.)
        boolean autoStarting = DriverStation.isAutonomousEnabled()
                && edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - lastDisabled_s < 0.5;
        if (DriverStation.isEnabled() && !autoStarting) {
            return;
        }
        placeSeq++;
        if (pigeon != null) {
            pigeon.getSimState().setRawYaw(pose.getRotation().getDegrees());
            pigeon.getSimState().setAngularVelocityZ(0.0);
        }
        placePub.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians(), placeSeq});
        nt.flush();
    }

    /** Call from Robot.simulationPeriodic: Driver Station mode from the simulator page, connection, true pose. */
    public void periodic() {
        if (!DriverStation.isEnabled()) {
            lastDisabled_s = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        }
        // "Robot ready" on the simulator page: the robot has just been turned on in its start configuration --
        // every motor's position reads 0, as after a real power-on
        double ready = readySub.get();
        if (ready != readySeen) {
            readySeen = ready;
            synchronized (this) {
                for (SimMotor m : motors) {
                    m.leader.setPosition(0.0);
                    for (TalonFX f : m.followers) {
                        f.setPosition(0.0);
                    }
                }
            }
        }
        String mode = dsModeSub.get();
        if (!mode.isEmpty() && !mode.equals(dsMode)) {
            dsMode = mode;
            DriverStationSim.setDsAttached(true);
            DriverStationSim.setAutonomous(mode.equals("auto"));
            DriverStationSim.setTest(false);
            DriverStationSim.setEnabled(!mode.equals("disabled"));
            DriverStationSim.notifyNewData();
        }
        robotStatePub.set(!DriverStation.isEnabled() ? "disabled" : DriverStation.isAutonomous() ? "auto"
                : DriverStation.isTest() ? "test" : "teleop");
        Logger.recordOutput("PhysicsSim/Connected", connected);
        double[] p = truthPoseSub.get();
        if (p.length >= 3) {
            Logger.recordOutput("PhysicsSim/TruthPose", new Pose2d(p[0], p[1], new Rotation2d(p[2])));
        }
    }

    public boolean isConnected() {
        return connected;
    }
}
