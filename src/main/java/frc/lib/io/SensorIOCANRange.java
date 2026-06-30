package frc.lib.io;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class SensorIOCANRange implements SensorIO {

  public static class SensorIOCANRangeConfig {
    public final int canId;
    public final String canBus;
    public double maxDistanceMeters = 0.5;
    public double minSignalStrength = 2500;
    public double proximityThreshold = 0.4;
    public double proximityHysteresis = 0.01;
    public double fovCenterXDeg = 0.0;
    public double fovCenterYDeg = 0.0;
    public double fovRangeXDeg = 27.0;
    public double fovRangeYDeg = 6.75;
    public double updateFrequencyHz = 50.0;

    public SensorIOCANRangeConfig(int canId, String canBus) {
      this.canId = canId;
      this.canBus = canBus;
    }

    public SensorIOCANRangeConfig withMaxDistance(double maxDistanceMeters) {
      this.maxDistanceMeters = maxDistanceMeters;
      return this;
    }

    public SensorIOCANRangeConfig withMinSignalStrength(double minSignalStrength) {
      this.minSignalStrength = minSignalStrength;
      return this;
    }

    public SensorIOCANRangeConfig withProximityThreshold(double threshold, double hysteresis) {
      this.proximityThreshold = threshold;
      this.proximityHysteresis = hysteresis;
      return this;
    }

    public SensorIOCANRangeConfig withFov(
        double centerXDeg, double centerYDeg, double rangeXDeg, double rangeYDeg) {
      this.fovCenterXDeg = centerXDeg;
      this.fovCenterYDeg = centerYDeg;
      this.fovRangeXDeg = rangeXDeg;
      this.fovRangeYDeg = rangeYDeg;
      return this;
    }
  }

  private final CANrange canrange;
  private final SensorIOCANRangeConfig config;
  private final Debouncer triggerDebouncer;
  private final Debouncer connectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final StatusSignal<Distance> distance;
  private final StatusSignal<Double> signalStrength;
  private final StatusSignal<Boolean> isDetected;

  public SensorIOCANRange(SensorIOCANRangeConfig config, double debounceSecs) {
    this.config = config;
    this.canrange = new CANrange(config.canId, config.canBus);
    this.triggerDebouncer = new Debouncer(debounceSecs, Debouncer.DebounceType.kBoth);

    var canrangeConfig =
        new CANrangeConfiguration()
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withMinSignalStrengthForValidMeasurement(config.minSignalStrength)
                    .withProximityHysteresis(config.proximityHysteresis)
                    .withProximityThreshold(config.proximityThreshold))
            .withToFParams(
                new ToFParamsConfigs()
                    .withUpdateFrequency(config.updateFrequencyHz)
                    .withUpdateMode(UpdateModeValue.ShortRangeUserFreq))
            .withFovParams(
                new FovParamsConfigs()
                    .withFOVCenterX(Units.Degrees.of(config.fovCenterXDeg))
                    .withFOVCenterY(Units.Degrees.of(config.fovCenterYDeg))
                    .withFOVRangeX(Units.Degrees.of(config.fovRangeXDeg))
                    .withFOVRangeY(Units.Degrees.of(config.fovRangeYDeg)));

    tryUntilOk(5, () -> canrange.getConfigurator().apply(canrangeConfig, 0.25));

    distance = canrange.getDistance();
    signalStrength = canrange.getSignalStrength();
    isDetected = canrange.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(
        config.updateFrequencyHz, distance, signalStrength, isDetected);
    canrange.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(distance, signalStrength, isDetected);

    inputs.connected = connectedDebouncer.calculate(status.isOK());
    inputs.distanceMeters = distance.getValueAsDouble(); // meters (Phoenix6 SI default)
    inputs.signalStrength = signalStrength.getValueAsDouble();
    inputs.isDetected = Boolean.TRUE.equals(isDetected.getValue());

    // Only trigger if the hardware proximity fires AND distance is within our configured max
    boolean withinRange = inputs.isDetected && inputs.distanceMeters <= config.maxDistanceMeters;
    inputs.isTriggered = triggerDebouncer.calculate(withinRange);
  }
}
