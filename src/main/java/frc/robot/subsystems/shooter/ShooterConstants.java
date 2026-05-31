// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.io.MotorIOPhoenix6.ClosedLoopOutput;
import frc.lib.io.MotorIOPhoenix6.GravityMode;
import frc.lib.io.MotorIOPhoenix6.MotorIOPhoenix6Config;
import frc.lib.io.MotorIOPhoenix6.NeutralMode;
import frc.lib.io.MotorIOSim.MotorIOSimConfig;

public final class ShooterConstants {
  public static final String CAN_BUS = "";

  public static final int DRUM_LEADER_ID = 20;
  public static final int DRUM_FOLLOWER_ID = 21;
  public static final int HOOD_ID = 22;
  public static final int HOLDER_ID = 23;

  public static final double DRUM_ROTOR_TO_MECHANISM_RATIO = 1.0;
  public static final double HOOD_ROTOR_TO_MECHANISM_RATIO = 50.0;
  public static final double HOLDER_ROTOR_TO_MECHANISM_RATIO = 1.0;

  public static final double DRUM_VELOCITY_TOLERANCE_RPS = 2.0;
  public static final double HOOD_ANGLE_TOLERANCE_RAD = Math.toRadians(1.0);
  public static final double HOLDER_VELOCITY_TOLERANCE_RPS = 2.0;

  public static final double DRUM_IDLE_RPS = 0.0;
  public static final double DRUM_CLOSE_RPS = 60.0;
  public static final double DRUM_FAR_RPS = 85.0;

  public static final Rotation2d HOOD_HOME_ANGLE = Rotation2d.kZero;
  public static final Rotation2d HOOD_CLOSE_ANGLE = Rotation2d.fromDegrees(18.0);
  public static final Rotation2d HOOD_FAR_ANGLE = Rotation2d.fromDegrees(34.0);

  public static final double HOOD_HOMING_VOLTS = -1.5;
  public static final double HOOD_HOMING_STATOR_CURRENT_AMPS = 25.0;
  public static final double HOOD_HOMING_DEBOUNCE_SECS = 0.25;
  public static final double HOOD_HOMING_TIMEOUT_SECS = 3.0;

  public static final double HOLDER_INTAKE_VOLTS = 4.0;
  public static final double HOLDER_HOLD_VOLTS = 0.8;
  public static final double HOLDER_FEED_VOLTS = 8.0;
  public static final double HOLDER_FEED_RPS = 25.0;

  public static final MotorIOPhoenix6Config DRUM_CONFIG =
      new MotorIOPhoenix6Config(DRUM_LEADER_ID, CAN_BUS)
          .withRotorToMechanismRatio(DRUM_ROTOR_TO_MECHANISM_RATIO)
          .withInverted(false)
          .withNeutralMode(NeutralMode.COAST)
          .withCurrentLimits(40.0, 80.0)
          .withSlot0(0.1, 0.0, 0.0, 0.0, 0.12, 0.0, 0.0)
          .withClosedLoopOutput(ClosedLoopOutput.VOLTAGE)
          .withFollower(DRUM_FOLLOWER_ID, true);

  public static final MotorIOPhoenix6Config HOOD_CONFIG =
      new MotorIOPhoenix6Config(HOOD_ID, CAN_BUS)
          .withRotorToMechanismRatio(HOOD_ROTOR_TO_MECHANISM_RATIO)
          .withInverted(false)
          .withNeutralMode(NeutralMode.BRAKE)
          .withCurrentLimits(30.0, 60.0)
          .withSlot0(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
          .withGravityMode(GravityMode.ARM_COSINE)
          .withMotionMagic(2.0, 4.0, 40.0);

  public static final MotorIOPhoenix6Config HOLDER_CONFIG =
      new MotorIOPhoenix6Config(HOLDER_ID, CAN_BUS)
          .withRotorToMechanismRatio(HOLDER_ROTOR_TO_MECHANISM_RATIO)
          .withInverted(false)
          .withNeutralMode(NeutralMode.BRAKE)
          .withCurrentLimits(30.0, 60.0)
          .withSlot0(0.1, 0.0, 0.0, 0.0, 0.12, 0.0, 0.0)
          .withClosedLoopOutput(ClosedLoopOutput.VOLTAGE);

  public static final MotorIOSimConfig DRUM_SIM_CONFIG =
      new MotorIOSimConfig()
          .withGearbox(DCMotor.getKrakenX60Foc(2))
          .withGearing(DRUM_ROTOR_TO_MECHANISM_RATIO)
          .withMomentOfInertia(0.004)
          .withVelocityGains(0.12, 0.0);

  public static final MotorIOSimConfig HOOD_SIM_CONFIG =
      new MotorIOSimConfig()
          .withGearbox(DCMotor.getKrakenX60Foc(1))
          .withGearing(HOOD_ROTOR_TO_MECHANISM_RATIO)
          .withMomentOfInertia(0.02)
          .withPositionGains(8.0, 0.0);

  public static final MotorIOSimConfig HOLDER_SIM_CONFIG =
      new MotorIOSimConfig()
          .withGearbox(DCMotor.getKrakenX60Foc(1))
          .withGearing(HOLDER_ROTOR_TO_MECHANISM_RATIO)
          .withMomentOfInertia(0.003)
          .withVelocityGains(0.12, 0.0);

  private ShooterConstants() {}
}
