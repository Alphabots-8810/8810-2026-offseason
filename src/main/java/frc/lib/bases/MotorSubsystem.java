// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.bases;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class MotorSubsystem extends SubsystemBase {
  protected final MotorIO io;
  protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();

  private final String logKey;
  private final Alert disconnectedAlert;

  public MotorSubsystem(MotorIO io, String logKey, String disconnectedAlertText) {
    this(io, logKey, disconnectedAlertText, false);
  }

  public MotorSubsystem(
      MotorIO io, String logKey, String disconnectedAlertText, boolean zeroOnInit) {
    this.io = io;
    this.logKey = logKey;
    disconnectedAlert = new Alert(disconnectedAlertText, AlertType.kError);
    if (zeroOnInit) io.setEncoderPositionRot(0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);
    disconnectedAlert.set(!inputs.connected);
  }

  public void setCurrent(double torqueCurrent) {
    io.setCurrent(torqueCurrent);
  }

  public double getPositionRot() {
    return inputs.positionRad / (2.0 * Math.PI);
  }

  public double getVelocityRotPerSec() {
    return inputs.velocityRadPerSec / (2.0 * Math.PI);
  }

  public double getVelocitySetpointRotPerSec() {
    return inputs.velocitySetpointRadPerSec / (2.0 * Math.PI);
  }

  public double getPositionSetpointRot() {
    return inputs.positionSetpointRad / (2.0 * Math.PI);
  }

  public double getStatorCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  public void setV(double volts) {
    io.setVoltage(volts);
  }

  public void setVelocityRotPerSec(double velocityRotPerSec) {
    io.setVelocityRotPerSec(velocityRotPerSec);
  }

  public void setPositionRot(double positionRot) {
    io.setPositionRot(positionRot);
  }

  public void setMotionMagicConstraints(
      double cruiseVelocityRotPerSec, double accelerationRotPerSecSq, double jerkRotPerSecCubed) {
    io.setMotionMagicConstraints(
        cruiseVelocityRotPerSec, accelerationRotPerSecSq, jerkRotPerSecCubed);
  }

  public void setEncoderPositionRot(double positionRot) {
    io.setEncoderPositionRot(positionRot);
  }

  /**
   * True when the mechanism is within tolerance of the active closed-loop setpoint. Compares
   * position (rot) in POSITION mode and velocity (rot/sec) in VELOCITY mode; false in open-loop
   * modes (VOLTAGE, CURRENT, DISABLED) where no setpoint exists.
   */
  public boolean isAtSetpoint(double toleranceRotOrRotPerSec) {
    switch (inputs.controlMode) {
      case POSITION:
        return Math.abs(getPositionRot() - getPositionSetpointRot()) < toleranceRotOrRotPerSec;
      case VELOCITY:
        return Math.abs(getVelocityRotPerSec() - getVelocitySetpointRotPerSec())
            < toleranceRotOrRotPerSec;
      default:
        return false;
    }
  }

  public void stop() {
    io.stop();
  }

  public void zeroPosition() {
    io.setEncoderPositionRot(0.0);
  }
}
