package frc.lib.io.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import frc.lib.bases.CameraSubsystem.CameraIOConfig;
import frc.lib.util.vision.VisionEstimate;
import java.util.List;
import java.util.Optional;

public abstract class CameraIO {

  protected final CameraIOConfig config;

  protected CameraIO(CameraIOConfig config) {
    this.config = config;
  }

  public String getName() {
    return config.name;
  }

  public boolean isConnected() {
    return false;
  }

  /** Returns the pose estimates produced since the last update, if any. */
  public Optional<List<VisionEstimate>> getLastEstimates() {
    return Optional.empty();
  }

  /** Returns the standard deviations to apply to an estimate, scaled by its target area. */
  public Vector<N3> getEstimateStdDevs(VisionEstimate estimate) {
    double xyStdDev = config.taToXYStdDevMeters.get(Math.max(0.0, estimate.getTargetArea()));
    return VecBuilder.fill(xyStdDev, xyStdDev, config.thetaStdDevRad);
  }

  /** Called once per loop from the subsystem periodic. */
  public void update() {}
}
