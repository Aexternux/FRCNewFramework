package frc.robot.sim;

import com.ctre.phoenix6.unmanaged.Unmanaged;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.devices.gyro.GyroIOInputsAutoLogged;
import frc.robot.devices.gyro.GyroIOPhysicsSim;
import frc.robot.devices.motor.MotorConfig;
import frc.robot.devices.motor.MotorIOInputsAutoLogged;
import frc.robot.devices.motor.MotorIOPhysicsSim;
import frc.robot.devices.swervemodule.SwerveModuleIOInputsAutoLogged;
import frc.robot.devices.swervemodule.SwerveModuleIOPhysicsSim;
import frc.robot.generated.TunerConstants;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

/**
 * End-to-end check of the physics link: the robot's own swerve module, gyro and motor IO, on
 * Phoenix's simulated firmware, closing their loops through the CAD physics simulator.
 *
 * <p>Needs the simulator page open on the robot and connected to NetworkTables (127.0.0.1), so it
 * only runs when PHYSICS_LINK=1. It serves NetworkTables itself.
 */
public class PhysicsSimLinkTest {

    private static final double HOOD_RATIO = 45.39;

    private static void run(double seconds, Runnable each) throws InterruptedException {
        long end = System.nanoTime() + (long) (seconds * 1e9);
        while (System.nanoTime() < end) {
            Unmanaged.feedEnable(100);
            each.run();
            Thread.sleep(20);
        }
    }

    @Test
    public void closedLoopsThroughThePhysics() throws InterruptedException {
        Assumptions.assumeTrue("1".equals(System.getenv("PHYSICS_LINK")));
        // robot enabled, as the Driver Station would (Phoenix's simulated devices follow it)
        edu.wpi.first.hal.HAL.initialize(500, 0);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setDsAttached(true);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(true);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
        NetworkTableInstance.getDefault().startServer();

        var constants = new com.ctre.phoenix6.swerve.SwerveModuleConstants[] {
            TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight};
        SwerveModuleIOPhysicsSim[] modules = new SwerveModuleIOPhysicsSim[4];
        SwerveModuleIOInputsAutoLogged[] in = new SwerveModuleIOInputsAutoLogged[4];
        for (int i = 0; i < 4; i++) {
            @SuppressWarnings("unchecked")
            var c = (com.ctre.phoenix6.swerve.SwerveModuleConstants<
                    com.ctre.phoenix6.configs.TalonFXConfiguration, com.ctre.phoenix6.configs.TalonFXConfiguration,
                    com.ctre.phoenix6.configs.CANcoderConfiguration>) constants[i];
            modules[i] = new SwerveModuleIOPhysicsSim(i, c);
            in[i] = new SwerveModuleIOInputsAutoLogged();
        }
        GyroIOPhysicsSim gyro = new GyroIOPhysicsSim();
        GyroIOInputsAutoLogged gin = new GyroIOInputsAutoLogged();

        // the hood: Motion Magic in mechanism rotations, as Motor would send it
        MotorConfig hoodCfg = new MotorConfig(30)
                .withSensorToMechanismRatio(HOOD_RATIO)
                .withFFGains(0.1, 0.12 * HOOD_RATIO, 0.0, 0.0)
                .withPIDGains(80.0, 0.0, 1.0, MotorConfig.GravityType.NONE)
                .withMotionMagic(1.0, 4.0, 0.0)
                .withMeasurementUnit(MotorConfig.MeasurementUnit.ROTATIONS);
        MotorIOPhysicsSim hood = new MotorIOPhysicsSim("Shooter/Hood", hoodCfg);
        MotorIOInputsAutoLogged hin = new MotorIOInputsAutoLogged();

        Runnable read = () -> {
            for (int i = 0; i < 4; i++) {
                modules[i].updateInputs(in[i]);
            }
            gyro.updateInputs(gin);
            hood.updateInputs(hin);
        };

        System.out.println("LINK waiting for the simulator...");
        long t0 = System.currentTimeMillis();
        while (!PhysicsSim.getInstance().isConnected() && System.currentTimeMillis() - t0 < 60000) {
            Unmanaged.feedEnable(100);
            Thread.sleep(50);
        }
        System.out.println("LINK connected: " + PhysicsSim.getInstance().isConnected());
        Assumptions.assumeTrue(PhysicsSim.getInstance().isConnected());

        run(1.0, read);
        System.out.println("LINK start: steer " + angles(in) + " gyro " + gin.yaw_Rot2d.getDegrees());

        // 1. steer every module to 90 deg (position loop on the fused CANcoder)
        run(2.0, () -> {
            read.run();
            for (var m : modules) {
                m.setTurnPosition(Rotation2d.fromDegrees(90));
                m.setDriveVelocity(0);
            }
        });
        System.out.println("LINK steer to 90: CANcoders " + angles(in));

        // 2. all modules to 0 deg, drive 20 rad/s (velocity loop on the drive rotor)
        run(3.0, () -> {
            read.run();
            for (var m : modules) {
                m.setTurnPosition(Rotation2d.fromDegrees(0));
                m.setDriveVelocity(20.0);
            }
        });
        StringBuilder v = new StringBuilder();
        for (var x : in) {
            v.append(String.format("%.2f ", x.driveVel_rps * 2 * Math.PI));
        }
        System.out.println("LINK drive 20 rad/s: measured " + v + "rad/s, CANcoders " + angles(in) + " gyro "
                + String.format("%.1f", gin.yaw_Rot2d.getDegrees()));

        // 3. rotate in place: modules tangent, drive 15 rad/s
        run(3.0, () -> {
            read.run();
            double[] tangent = {135, 45, -135, -45};   // FL, FR, BL, BR: perpendicular to the centre, CCW
            for (int i = 0; i < 4; i++) {
                modules[i].setTurnPosition(Rotation2d.fromDegrees(tangent[i]));
                modules[i].setDriveVelocity(15.0);
            }
        });
        System.out.println("LINK rotate: gyro " + String.format("%.1f deg, %.2f rad/s", gin.yaw_Rot2d.getDegrees(),
                gin.yawVel_radps) + " CANcoders " + angles(in));
        run(1.0, () -> {
            read.run();
            for (var m : modules) {
                m.setDriveVelocity(0);
            }
        });

        // 4. hood: Motion Magic to 0.03 mechanism rotations (10.8 deg) and back
        double start = hin.position;
        run(3.0, () -> {
            read.run();
            hood.setMotionMagic(start - 0.03);
        });
        System.out.println(String.format("LINK hood motion magic -0.030 rot: %.4f -> %.4f (%.2f V, %.0f A)", start,
                hin.position, hin.appliedVolts, hin.statorCurrent));
        run(2.0, () -> {
            read.run();
            hood.setMotionMagic(start);
        });
        System.out.println(String.format("LINK hood back: %.4f", hin.position));
    }

    private static String angles(SwerveModuleIOInputsAutoLogged[] in) {
        StringBuilder s = new StringBuilder();
        for (var x : in) {
            s.append(String.format("%.1f ", x.steerAbsPos_Rot2d.getDegrees()));
        }
        return s.toString().trim();
    }
}
