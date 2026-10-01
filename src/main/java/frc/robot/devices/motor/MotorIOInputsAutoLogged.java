package frc.robot.devices.motor;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MotorIOInputsAutoLogged extends MotorIO.MotorIOInputs
    implements LoggableInputs, Cloneable {

    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
        table.put("Position", position);
        table.put("Velocity", velocity);
        table.put("Acceleration", acceleration);
        table.put("AppliedVolts", appliedVolts);
        table.put("SupplyCurrent", supplyCurrent);
        table.put("StatorCurrent", statorCurrent);
        table.put("TempC", tempC);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        position = table.get("Position", position);
        velocity = table.get("Velocity", velocity);
        acceleration = table.get("Acceleration", acceleration);
        appliedVolts = table.get("AppliedVolts", appliedVolts);
        supplyCurrent = table.get("SupplyCurrent", supplyCurrent);
        statorCurrent = table.get("StatorCurrent", statorCurrent);
        tempC = table.get("TempC", tempC);
    }

    @Override
    public MotorIOInputsAutoLogged clone() {
        MotorIOInputsAutoLogged copy = new MotorIOInputsAutoLogged();

        copy.connected = this.connected;
        copy.position = this.position;
        copy.velocity = this.velocity;
        copy.acceleration = this.acceleration;
        copy.appliedVolts = this.appliedVolts;
        copy.supplyCurrent = this.supplyCurrent;
        copy.statorCurrent = this.statorCurrent;
        copy.tempC = this.tempC;

        return copy;
    }
}