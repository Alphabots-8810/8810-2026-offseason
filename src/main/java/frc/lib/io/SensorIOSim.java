package frc.lib.io;

import edu.wpi.first.math.filter.Debouncer;
import java.util.function.BooleanSupplier;

public class SensorIOSim implements SensorIO {
  private final BooleanSupplier detectedSupplier;
  private final Debouncer debouncer;

  public SensorIOSim(BooleanSupplier detectedSupplier, double debounceSecs) {
    this.detectedSupplier = detectedSupplier;
    this.debouncer = new Debouncer(debounceSecs, Debouncer.DebounceType.kBoth);
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.connected = true;
    inputs.distanceMeters = 0.0;
    inputs.signalStrength = 0.0;
    boolean detected = detectedSupplier.getAsBoolean();
    inputs.isDetected = detected;
    inputs.isTriggered = debouncer.calculate(detected);
  }
}
