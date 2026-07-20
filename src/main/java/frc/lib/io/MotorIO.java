// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.io;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {
  public static enum ControlMode {
    DISABLED,
    VOLTAGE,
    VELOCITY,
    POSITION,
    CURRENT
  }

  @AutoLog
  public static class MotorIOInputs {
    public boolean connected = false;
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    /**
     * Per-motor currents, leader first then followers in config order. On a healthy multi-motor
     * mechanism these track each other; a spread points at an asymmetrical mechanical or electrical
     * problem.
     */
    public double[] supplyCurrentAmps = new double[] {};

    public double[] statorCurrentAmps = new double[] {};
    public double tempCelsius = 0.0;

    public ControlMode controlMode = ControlMode.DISABLED;
    public double voltageSetpoint = 0.0;
    public double velocitySetpointRadPerSec = 0.0;
    public double positionSetpointRad = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MotorIOInputs inputs) {}

  /** Runs the motor at the specified voltage. */
  public default void setVoltage(double volts) {}

  public default void setCurrent(double current) {}

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

  public default double getPositionRot() {
    return 0.0;
  }

  public default double getVelocityRotPerSec() {
    return 0.0;
  }

  public default void setVelocityRotPerSec(double velocityRotPerSec) {}

  public default void setPositionRot(double positionRot) {}

  public default void setEncoderPositionRot(double positionRot) {}

  /** Updates the Dynamic Motion Magic cruise velocity, acceleration, and jerk for future moves. */
  public default void setMotionMagicConstraints(
      double cruiseVelocityRotPerSec, double accelerationRotPerSecSq, double jerkRotPerSecCubed) {}

  /** Sets the motor torque current limit in amps (forward direction). */
  public default void setMotorTorqueCurrentLimit(double amps) {}
}
