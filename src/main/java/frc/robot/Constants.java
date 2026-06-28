// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final class VisionConstants {
    // TA (target area, 0–100) → XY std dev in metres. Larger TA = closer tag = more trust.
    public static final InterpolatingDoubleTreeMap taToXYStdDevMeters =
        new InterpolatingDoubleTreeMap();

    static {
      taToXYStdDevMeters.put(0.0, 4.00); // barely visible — almost no trust
      taToXYStdDevMeters.put(0.5, 0.50);
      taToXYStdDevMeters.put(1.0, 0.30);
      taToXYStdDevMeters.put(3.0, 0.10); // close tag — high trust
    }

    // Heading is always locked to the gyro, so give rotation a huge std dev.
    public static final double thetaStdDevRad = 9999.0;

    private VisionConstants() {}
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
