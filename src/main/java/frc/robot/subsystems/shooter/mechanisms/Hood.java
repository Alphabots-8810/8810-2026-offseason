// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter.mechanisms;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.bases.MotorSubsystem;
import frc.lib.io.MotorIO;
import frc.lib.io.MotorIOPhoenix6;
import frc.lib.io.MotorIOSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Hood extends MotorSubsystem {
  public static enum State {
    HOMING,
    IDLE,
    MOVING,
    AT_TARGET
  }

  private final Debouncer homingCurrentDebouncer =
      new Debouncer(ShooterConstants.HOOD_HOMING_DEBOUNCE_SECS, Debouncer.DebounceType.kRising);
  private final Timer homingTimer = new Timer();

  private State state = State.IDLE;
  private double goalAngleRad = ShooterConstants.HOOD_HOME_ANGLE.getRadians();
  private boolean homingTimedOut = false;

  public Hood() {
    this(createIO());
  }

  public Hood(MotorIO io) {
    super(io, "Shooter/Hood", "Disconnected shooter hood motor.");
  }

  @Override
  public void periodic() {
    super.periodic();

    switch (state) {
      case HOMING:
        setVoltageOut(ShooterConstants.HOOD_HOMING_VOLTS);
        boolean homed =
            homingCurrentDebouncer.calculate(
                getStatorCurrentAmps() >= ShooterConstants.HOOD_HOMING_STATOR_CURRENT_AMPS);
        homingTimedOut = homingTimer.hasElapsed(ShooterConstants.HOOD_HOMING_TIMEOUT_SECS);
        if (homed) {
          setEncoderPositionRad(ShooterConstants.HOOD_HOME_ANGLE.getRadians());
          goalAngleRad = ShooterConstants.HOOD_HOME_ANGLE.getRadians();
          stopMotor();
          state = State.IDLE;
        } else if (homingTimedOut) {
          stopMotor();
          state = State.IDLE;
        }
        break;

      case IDLE:
        stopMotor();
        break;

      case MOVING:
      case AT_TARGET:
        setPositionRad(goalAngleRad);
        state = atGoal() ? State.AT_TARGET : State.MOVING;
        break;
    }

    Logger.recordOutput("Shooter/Hood/State", state);
    Logger.recordOutput("Shooter/Hood/GoalRad", goalAngleRad);
    Logger.recordOutput("Shooter/Hood/HomingTimedOut", homingTimedOut);
  }

  public void setGoal(Rotation2d angle) {
    setAngle(angle);
  }

  public void setAngle(Rotation2d angle) {
    setAngleRad(angle.getRadians());
  }

  public void setAngleRad(double angleRad) {
    goalAngleRad = angleRad;
    state = atGoal() ? State.AT_TARGET : State.MOVING;
  }

  public void startHoming() {
    homingTimedOut = false;
    homingTimer.reset();
    homingTimer.start();
    homingCurrentDebouncer.calculate(false);
    state = State.HOMING;
  }

  public Command runHomingCommand() {
    return Commands.runOnce(this::startHoming, this)
        .andThen(Commands.waitUntil(() -> state == State.IDLE))
        .finallyDo(this::stop);
  }

  public void stop() {
    stopMotor();
    state = State.IDLE;
  }

  public boolean atGoal() {
    return Math.abs(goalAngleRad - getAngle().getRadians())
        <= ShooterConstants.HOOD_ANGLE_TOLERANCE_RAD;
  }

  public Rotation2d getAngle() {
    return new Rotation2d(getPositionRad());
  }

  public State getState() {
    return state;
  }

  private static MotorIO createIO() {
    return switch (Constants.currentMode) {
      case REAL -> new MotorIOPhoenix6(ShooterConstants.HOOD_CONFIG);
      case SIM -> new MotorIOSim(ShooterConstants.HOOD_SIM_CONFIG);
      case REPLAY -> new MotorIO() {};
    };
  }
}
