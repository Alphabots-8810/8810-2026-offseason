// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.mechanisms.Drum;
import frc.robot.subsystems.shooter.mechanisms.Holder;
import frc.robot.subsystems.shooter.mechanisms.Hood;
import org.littletonrobotics.junction.Logger;

public class Shooter {
  public static enum State {
    IDLE,
    HOMING,
    SPIN_UP,
    READY,
    SHOOT
  }

  private final Drum drum;
  private final Hood hood;
  private final Holder holder;

  private State state = State.IDLE;
  private boolean readyToShoot = false;

  public Shooter() {
    this(new Drum(), new Hood(), new Holder());
  }

  public Shooter(Drum drum, Hood hood, Holder holder) {
    this.drum = drum;
    this.hood = hood;
    this.holder = holder;
    logState();
  }

  public Command shootCommand(double targetRps, Rotation2d hoodAngle) {
    return Commands.run(() -> runShootStateMachine(targetRps, hoodAngle), drum, hood, holder)
        .beforeStarting(
            () -> {
              state = State.SPIN_UP;
              readyToShoot = false;
              logState();
            })
        .finallyDo(this::resetToIdle);
  }

  public Command idleCommand() {
    return Commands.runOnce(this::resetToIdle, drum, hood, holder);
  }

  public Command homeCommand() {
    return Commands.run(this::runHomeStateMachine, drum, hood, holder)
        .beforeStarting(
            () -> {
              state = State.HOMING;
              readyToShoot = false;
              drum.stop();
              holder.stop();
              hood.startHoming();
              logState();
            })
        .until(() -> hood.getState() == Hood.State.IDLE)
        .finallyDo(this::resetToIdle);
  }

  public State getState() {
    return state;
  }

  public boolean readyToShoot() {
    return readyToShoot;
  }

  public Drum getDrum() {
    return drum;
  }

  public Hood getHood() {
    return hood;
  }

  public Holder getHolder() {
    return holder;
  }

  private void runShootStateMachine(double targetRps, Rotation2d hoodAngle) {
    switch (state) {
      case IDLE:
        state = State.SPIN_UP;
        break;

      case SPIN_UP:
        commandSpinUp(targetRps, hoodAngle);
        if (drum.getState() == Drum.State.AT_SPEED && hood.getState() == Hood.State.AT_TARGET) {
          state = State.READY;
        }
        break;

      case READY:
        commandSpinUp(targetRps, hoodAngle);
        if (drum.getState() == Drum.State.AT_SPEED && hood.getState() == Hood.State.AT_TARGET) {
          state = State.SHOOT;
        } else {
          state = State.SPIN_UP;
        }
        break;

      case SHOOT:
        commandSpinUp(targetRps, hoodAngle);
        holder.feed();
        if (drum.getState() != Drum.State.AT_SPEED || hood.getState() != Hood.State.AT_TARGET) {
          state = State.SPIN_UP;
        }
        break;

      case HOMING:
        runHomeStateMachine();
        break;
    }

    readyToShoot = state == State.READY || state == State.SHOOT;
    logState();
  }

  private void runHomeStateMachine() {
    drum.stop();
    holder.stop();
    if (hood.getState() == Hood.State.IDLE) {
      state = State.IDLE;
    }
    readyToShoot = false;
    logState();
  }

  private void commandSpinUp(double targetRps, Rotation2d hoodAngle) {
    drum.setVelocity(targetRps);
    hood.setAngle(hoodAngle);
    holder.hold();
  }

  private void resetToIdle() {
    state = State.IDLE;
    readyToShoot = false;
    drum.stop();
    hood.stop();
    holder.stop();
    logState();
  }

  private void logState() {
    Logger.recordOutput("Shooter/State", state);
    Logger.recordOutput("Shooter/ReadyToShoot", readyToShoot);
  }
}
