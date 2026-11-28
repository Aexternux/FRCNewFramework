package frc.robot.devices.swervemodule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.generated.TunerConstants;
import frc.robot.sim.PhysicsSim;

import java.util.List;

/**
 * The robot's own {@link SwerveModuleIOTalonFX}, running against the CAD physics simulator.
 *
 * <p>Same drive and steer TalonFXs, CANcoder, configs and control requests as on the robot. The
 * physics drives the CAD module's wheel on the carpet (with grip and slip) and turns it, and reports
 * the drive rotor, the steer rotor and the CANcoder angle back, through {@link PhysicsSim}.
 */
public class SwerveModuleIOPhysicsSim extends SwerveModuleIOTalonFX {

    public SwerveModuleIOPhysicsSim(int index,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
        super(constants);
        String bus = TunerConstants.DrivetrainConstants.CANBusName;
        String name = "Drive/Module" + index;
        PhysicsSim sim = PhysicsSim.getInstance();
        sim.registerMotor(name + "/Drive", "drive", index, bus, getDriveTalon(), List.of(), List.of(),
                constants.DriveMotorInverted, true);
        sim.registerMotor(name + "/Steer", "steer", index, bus, getTurnTalon(), List.of(), List.of(),
                constants.SteerMotorInverted, true);
        sim.registerEncoder(bus, getCancoder(), constants.EncoderOffset, constants.EncoderInverted);
        sim.registerModule(index, bus, constants.DriveMotorId, constants.SteerMotorId, constants.EncoderId,
                constants.LocationX, constants.LocationY, constants.DriveMotorGearRatio,
                constants.SteerMotorGearRatio, constants.WheelRadius, constants.CouplingGearRatio);
    }

    @Override
    public void setBrake(boolean brake) {
        super.setBrake(brake);
        PhysicsSim.getInstance().setBrake(getDriveTalon(), brake);
        PhysicsSim.getInstance().setBrake(getTurnTalon(), brake);
    }
}
