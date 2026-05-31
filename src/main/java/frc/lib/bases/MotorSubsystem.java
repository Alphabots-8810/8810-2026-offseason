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
    this.io = io;
    this.logKey = logKey;
    disconnectedAlert = new Alert(disconnectedAlertText, AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(logKey, inputs);
    disconnectedAlert.set(!inputs.connected);
  }

  protected double getPositionRad() {
    return inputs.positionRad;
  }

  protected double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }

  protected double getStatorCurrentAmps() {
    return inputs.statorCurrentAmps;
  }

  protected void setVoltageOut(double volts) {
    io.setVoltage(volts);
  }

  protected void setVelocityRadPerSec(double velocityRadPerSec) {
    io.setVelocity(velocityRadPerSec);
  }

  protected void setPositionRad(double positionRad) {
    io.setPosition(positionRad);
  }

  protected void setEncoderPositionRad(double positionRad) {
    io.setEncoderPosition(positionRad);
  }

  protected void stopMotor() {
    io.stop();
  }
}
