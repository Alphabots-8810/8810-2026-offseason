// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.Hood;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.io.MotorIOPhoenix6.ClosedLoopOutput;
import frc.lib.io.MotorIOPhoenix6.MotorIOPhoenix6Config;
import frc.lib.io.MotorIOPhoenix6.NeutralMode;
import frc.lib.io.MotorIOSim.MotorIOSimConfig;

public final class HoodConstants {
  public static final String CAN_BUS = "";

  public static final int Hood_LEADER_ID = 30;
  public static final int Hood_FOLLOWER_ID = 31;

  public static final double Hood_ROTOR_TO_MECHANISM_RATIO = 1.0;

  public static final double Hood_INTAKE_VOLTS = 6.0;
  public static final double Hood_FEED_VOLTS = 8.0;
  public static final double Hood_REVERSE_VOLTS = -4.0;
  public static final double Hood_IDLE_VOLTS = 0.0;

  public static final MotorIOPhoenix6Config Hood_CONFIG =
      new MotorIOPhoenix6Config(Hood_LEADER_ID, CAN_BUS)
          .withRotorToMechanismRatio(Hood_ROTOR_TO_MECHANISM_RATIO)
          .withInverted(false)
          .withNeutralMode(NeutralMode.BRAKE)
          .withCurrentLimits(30.0, 60.0)
          .withSlot0(0.1, 0.0, 0.0, 0.0, 0.12, 0.0, 0.0)
          .withClosedLoopOutput(ClosedLoopOutput.VOLTAGE)
          .withFollower(Hood_FOLLOWER_ID, true);

  public static final MotorIOSimConfig Hood_SIM_CONFIG =
      new MotorIOSimConfig()
          .withGearbox(DCMotor.getKrakenX60Foc(2))
          .withGearing(Hood_ROTOR_TO_MECHANISM_RATIO)
          .withMomentOfInertia(0.003)
          .withVelocityGains(0.12, 0.0);

  private HoodConstants() {}
}
