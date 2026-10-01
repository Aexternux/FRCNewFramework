package frc.robot.devices.canrange;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.Constants;

public class CANrangeIOReal implements CANrangeIO {

    private final CANrange canrange;

    private final StatusSignal<Distance> distance;
    private final StatusSignal<Boolean> isDetected;
    private final StatusSignal<Double> signalStrength;
    private final StatusSignal<Double> ambientSignal;

    public CANrangeIOReal(CANrangeConfig config) {
        canrange = new CANrange(config.canId, config.canbus);

        CANrangeConfiguration cfg = new CANrangeConfiguration();
        cfg.ProximityParams.ProximityThreshold = config.proximityThreshold_m;
        cfg.ProximityParams.ProximityHysteresis = config.proximityHysteresis_m;
        cfg.ProximityParams.MinSignalStrengthForValidMeasurement = config.minSignalStrength;
        tryUntilOk(5, () -> canrange.getConfigurator().apply(cfg, 0.25));

        distance = canrange.getDistance();
        isDetected = canrange.getIsDetected();
        signalStrength = canrange.getSignalStrength();
        ambientSignal = canrange.getAmbientSignal();

        BaseStatusSignal.setUpdateFrequencyForAll(
                Constants.globalDelta_Hz, distance, isDetected, signalStrength, ambientSignal);
        canrange.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(CANrangeIOInputs inputs) {
        var status = BaseStatusSignal.refreshAll(distance, isDetected, signalStrength, ambientSignal);
        inputs.connected = status.isOK();
        inputs.distance_m = distance.getValueAsDouble();
        inputs.isDetected = isDetected.getValue();
        inputs.signalStrength = signalStrength.getValueAsDouble();
        inputs.ambientSignal = ambientSignal.getValueAsDouble();
    }
}
