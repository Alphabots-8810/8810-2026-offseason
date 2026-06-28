// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.IntakeRoller;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.io.MotorIOPhoenix6.ClosedLoopOutput;
import frc.lib.io.MotorIOPhoenix6.MotorIOPhoenix6Config;
import frc.lib.io.MotorIOPhoenix6.NeutralMode;
import frc.lib.io.MotorIOSim.MotorIOSimConfig;

public final class IntakeRollerConstants {
  public static final String CAN_BUS = "rio";

  public static final int IntakeRoller_L_ID = 20;
  public static final int IntakeRoller_R_ID = 21;

  public static final double IntakeRoller_ROTOR_TO_MECHANISM_RATIO = 1; // 33mm per rot
  // initial position: 0.55737m

  public static final MotorIOPhoenix6Config IntakeRoller_CONFIG =
      new MotorIOPhoenix6Config(IntakeRoller_L_ID, CAN_BUS)
          .withRotorToMechanismRatio(IntakeRoller_ROTOR_TO_MECHANISM_RATIO)
          .withInverted(true)
          .withNeutralMode(NeutralMode.COAST)
          .withCurrentLimits(30.0, 60.0)
          .withSlot0(10, 0.0, 0.0, 7, 0.0, 0.0, 0.0)
          .withClosedLoopOutput(ClosedLoopOutput.TORQUE_CURRENT_FOC)
          .withFollower(IntakeRoller_R_ID, true);

  public static final MotorIOSimConfig IntakeRoller_SIM_CONFIG =
      new MotorIOSimConfig()
          .withGearbox(DCMotor.getKrakenX60Foc(2))
          .withGearing(IntakeRoller_ROTOR_TO_MECHANISM_RATIO)
          .withMomentOfInertia(0.003)
          .withVelocityGains(0.12, 0.0);

  private IntakeRollerConstants() {}
}
