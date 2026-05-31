// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.io;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
  @AutoLog
  public static class MotorIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MotorIOInputs inputs) {}

  /** Runs the motor at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Runs the motor at the specified velocity in radians per second. */
  public default void setVelocity(double velocityRadPerSec) {}

  /** Runs the motor to the specified position in radians. */
  public default void setPosition(double positionRad) {}

  /** Sets the neutral mode. */
  public default void setNeutralMode(boolean brake) {}

  /** Resets the encoder position in radians. */
  public default void setEncoderPosition(double positionRad) {}

  /** Disables output to the motor. */
  public default void stop() {}
}
