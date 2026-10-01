package frc.robot.devices.canrange;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CANrangeIOInputsAutoLogged extends CANrangeIO.CANrangeIOInputs
    implements LoggableInputs, Cloneable {

    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
        table.put("Distance_m", distance_m);
        table.put("IsDetected", isDetected);
        table.put("SignalStrength", signalStrength);
        table.put("AmbientSignal", ambientSignal);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
        distance_m = table.get("Distance_m", distance_m);
        isDetected = table.get("IsDetected", isDetected);
        signalStrength = table.get("SignalStrength", signalStrength);
        ambientSignal = table.get("AmbientSignal", ambientSignal);
    }

    @Override
    public CANrangeIOInputsAutoLogged clone() {
        CANrangeIOInputsAutoLogged copy = new CANrangeIOInputsAutoLogged();

        copy.connected = this.connected;
        copy.distance_m = this.distance_m;
        copy.isDetected = this.isDetected;
        copy.signalStrength = this.signalStrength;
        copy.ambientSignal = this.ambientSignal;

        return copy;
    }
}