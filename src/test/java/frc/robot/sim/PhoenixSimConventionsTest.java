package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.unmanaged.Unmanaged;
import org.junit.jupiter.api.Test;

/**
 * Pins down how Phoenix 6's simulated devices map their sim state (what PhysicsSim reads and
 * writes) to what robot code sees. PhysicsSim's sign handling depends on exactly these facts.
 */
public class PhoenixSimConventionsTest {

    // robot enabled, as the Driver Station would (Phoenix's simulated devices follow it; without
    // this they flicker between enabled and disabled)
    @org.junit.jupiter.api.BeforeAll
    public static void enable() {
        edu.wpi.first.hal.HAL.initialize(500, 0);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setDsAttached(true);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled(true);
        edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData();
    }

    private static void settle() throws InterruptedException {
        for (int i = 0; i < 20; i++) {
            Unmanaged.feedEnable(100);
            Thread.sleep(10);
        }
    }

    @Test
    public void invertedTalonSimStateIsRaw() throws InterruptedException {
        TalonFX normal = new TalonFX(41);
        TalonFX inverted = new TalonFX(42);
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        normal.getConfigurator().apply(cfg);
        cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        inverted.getConfigurator().apply(cfg);
        normal.getSimState().setSupplyVoltage(12);
        inverted.getSimState().setSupplyVoltage(12);

        normal.setControl(new VoltageOut(6));
        inverted.setControl(new VoltageOut(6));
        normal.getSimState().setRawRotorPosition(1.0);
        inverted.getSimState().setRawRotorPosition(1.0);
        settle();
        BaseStatusSignal.refreshAll(normal.getPosition(), inverted.getPosition());

        System.out.println("CONVENTION normal: simVolts " + normal.getSimState().getMotorVoltage()
                + " pos " + normal.getPosition().getValueAsDouble());
        System.out.println("CONVENTION inverted: simVolts " + inverted.getSimState().getMotorVoltage()
                + " pos " + inverted.getPosition().getValueAsDouble());
        // (less than 6 V: at stall the simulated firmware's default current limit cuts it, as a real
        // Talon's would)
        double v = normal.getSimState().getMotorVoltage();
        assertEquals(true, v > 1.0);
        assertEquals(1.0, normal.getPosition().getValueAsDouble(), 0.01);
        // PhysicsSim assumes: an inverted motor's sim voltage and raw rotor are both mirrored
        assertEquals(-v, inverted.getSimState().getMotorVoltage(), 0.05);
        assertEquals(-1.0, inverted.getPosition().getValueAsDouble(), 0.01);
        normal.close();
        inverted.close();
    }

    @Test
    public void cancoderOffsetAndDirection() throws InterruptedException {
        CANcoder ccw = new CANcoder(43);
        CANcoder cw = new CANcoder(44);
        CANcoderConfiguration cfg = new CANcoderConfiguration();
        cfg.MagnetSensor.MagnetOffset = 0.1;
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        ccw.getConfigurator().apply(cfg);
        cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        cw.getConfigurator().apply(cfg);
        ccw.getSimState().setRawPosition(0.2);
        cw.getSimState().setRawPosition(0.2);
        settle();
        BaseStatusSignal.refreshAll(ccw.getAbsolutePosition(), cw.getAbsolutePosition());
        double a = ccw.getAbsolutePosition().getValueAsDouble();
        double b = cw.getAbsolutePosition().getValueAsDouble();
        System.out.println("CONVENTION cancoder raw 0.2 offset 0.1: ccw " + a + " cw " + b);
        // PhysicsSim assumes: reported = dir * raw + offset (dir = -1 for clockwise-positive)
        assertEquals(0.3, a, 0.005);
        assertEquals(-0.1, b, 0.005);
        ccw.close();
        cw.close();
    }

    @Test
    public void pigeonRawYaw() throws InterruptedException {
        Pigeon2 pigeon = new Pigeon2(45);
        pigeon.getSimState().setRawYaw(30.0);
        settle();
        pigeon.getYaw().refresh();
        System.out.println("CONVENTION pigeon raw yaw 30 -> " + pigeon.getYaw().getValueAsDouble());
        assertEquals(30.0, pigeon.getYaw().getValueAsDouble(), 0.1);
        pigeon.close();
    }
}
