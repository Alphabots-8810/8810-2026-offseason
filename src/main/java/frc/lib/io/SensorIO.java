package frc.lib.io;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {
  @AutoLog
  public static class SensorIOInputs {
    public boolean connected = false;
    public double distanceMeters = 0.0;
    public double signalStrength = 0.0;
    public boolean isDetected = false; // raw proximity from hardware threshold
    public boolean isTriggered = false; // debounced + max-distance gated
  }

  public default void updateInputs(SensorIOInputs inputs) {}
}
