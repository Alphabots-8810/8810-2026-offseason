package frc.lib.util.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;

/** A single robot pose estimate produced by a camera. */
public class VisionEstimate {

  private final Pose2d pose;
  private final double timestampSeconds;
  private final int[] tagIds;
  private double averageTagDistanceMeters = 0.0;
  private double ta = 0.0;

  public VisionEstimate(Pose2d pose, double timestampSeconds, int[] tagIds) {
    this.pose = pose;
    this.timestampSeconds = timestampSeconds;
    this.tagIds = tagIds;
  }

  public Pose2d getPose() {
    return pose;
  }

  public double getTimestampSeconds() {
    return timestampSeconds;
  }

  public int[] getTagIds() {
    return tagIds;
  }

  public int getTagCount() {
    return tagIds.length;
  }

  public VisionEstimate withAverageDistance(double averageTagDistanceMeters) {
    this.averageTagDistanceMeters = averageTagDistanceMeters;
    return this;
  }

  public double getAverageDistanceMeters() {
    return averageTagDistanceMeters;
  }

  /** Target area (0-100), used to scale standard deviations. Larger area = closer tag. */
  public VisionEstimate withTargetArea(double ta) {
    this.ta = ta;
    return this;
  }

  public double getTargetArea() {
    return ta;
  }

  public void log(String key) {
    Logger.recordOutput(key + "/Pose", pose);
    Logger.recordOutput(key + "/TimestampSeconds", timestampSeconds);
    Logger.recordOutput(key + "/TagIds", tagIds);
    Logger.recordOutput(key + "/AverageTagDistance", averageTagDistanceMeters);
    Logger.recordOutput(key + "/TA", ta);
  }
}
