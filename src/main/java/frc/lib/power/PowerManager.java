// Copyright (c) 2026 FRC 8810
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.lib.power;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized power budgeting ("省电模式"). Design based on 6328's 2026 FinanceDepartment
 * (Mechanical-Advantage/RobotCode2026Public) simplified for a stock Phoenix 6 + roboRIO stack, plus
 * 1678-style battery alerting (frc1678/C2026-Public BatteryChecker).
 *
 * <p>How it works, once per loop (called from {@link frc.robot.Robot#robotPeriodic()} after the
 * command scheduler so every subsystem has already reported its current draw):
 *
 * <ol>
 *   <li>Sum the supply currents reported by all subsystems via {@link #reportCurrent}.
 *   <li>Estimate the battery's rest (open-circuit) voltage by compensating the measured bus voltage
 *       for IR drop: vRest ≈ vBatt + iTotal * R0, low-pass filtered. This separates "battery is
 *       genuinely low" from "battery is momentarily sagging under load".
 *   <li>Compute the total current the battery can source without sagging below the brownout floor,
 *       and hand everything the mechanisms aren't using to the drivetrain as its supply-current
 *       budget. Mechanisms keep their fixed limits; the drivetrain is the shock absorber.
 *   <li>If the roboRIO actually browned out, cut the drive budget immediately and only ramp it back
 *       up slowly (asymmetric response, same as 6328).
 *   <li>Classify a power tier (NORMAL / CONSERVE / CRITICAL) from the rest-voltage estimate for
 *       mechanisms that want to shed comfort loads (e.g. stop idling the flywheel).
 * </ol>
 *
 * <p>The drivetrain consumes {@link #getDriveModuleSupplyLimitAmps()} (see {@code Drive.periodic});
 * mechanisms may consume {@link #getTier()} / {@link #shouldConserve()} in their commands. All
 * inputs and outputs are logged under "PowerManager/" for AdvantageScope, which doubles as a
 * per-mechanism power log for post-match analysis (same idea as 1678's PowerSlice).
 */
public class PowerManager {
  public enum PowerTier {
    NORMAL,
    CONSERVE,
    CRITICAL
  }

  @AutoLog
  public static class PowerManagerInputs {
    public double batteryVoltage = 12.0;
    public boolean brownedOut = false;
  }

  private static final double loopPeriodSecs = 0.02;
  private static final double quantizationAmps = 2.0;

  // Effective battery + wiring resistance seen from the roboRIO voltage measurement. Tune by
  // plotting PowerManager/RestVoltage in AdvantageScope: it should stay flat during hard
  // accelerations (if it dips with load, increase R0; if it bumps up with load, decrease R0).
  private static final LoggedTunableNumber batteryResistanceOhms =
      new LoggedTunableNumber("PowerManager/BatteryResistanceOhms", 0.020);
  // Rio, radio, and other loads that never report currents.
  private static final LoggedTunableNumber overheadAmps =
      new LoggedTunableNumber("PowerManager/OverheadAmps", 3.0);
  // Bus voltage we refuse to sag below. Keep above the roboRIO brownout trigger.
  private static final LoggedTunableNumber brownoutFloorVolts =
      new LoggedTunableNumber("PowerManager/BrownoutFloorVolts", 7.0);
  private static final LoggedTunableNumber budgetHeadroom =
      new LoggedTunableNumber("PowerManager/BudgetHeadroom", 0.9);
  private static final LoggedTunableNumber maxTotalBudgetAmps =
      new LoggedTunableNumber("PowerManager/MaxTotalBudgetAmps", 240.0);
  // Rest-voltage thresholds for the tier state machine (compare against IR-compensated rest
  // voltage, NOT the sagging bus voltage). 1678 alerts at 12.3/12.0 measured at rest.
  private static final LoggedTunableNumber conserveRestVolts =
      new LoggedTunableNumber("PowerManager/ConserveRestVolts", 12.1);
  private static final LoggedTunableNumber criticalRestVolts =
      new LoggedTunableNumber("PowerManager/CriticalRestVolts", 11.8);
  // Per-module drive supply limits. Max should match TunerConstants' configured supply limit.
  private static final LoggedTunableNumber driveModuleMinAmps =
      new LoggedTunableNumber("PowerManager/DriveModuleMinAmps", 15.0);
  private static final LoggedTunableNumber driveModuleMaxAmps =
      new LoggedTunableNumber("PowerManager/DriveModuleMaxAmps", 70.0);
  // Auto runs a fixed limit: autos are short, repeatable, and should not vary with battery state.
  private static final LoggedTunableNumber driveAutoModuleAmps =
      new LoggedTunableNumber("PowerManager/DriveAutoModuleAmps", 70.0);
  // How fast the drive budget is allowed to recover after a brownout.
  private static final LoggedTunableNumber brownoutRecoveryAmpsPerSec =
      new LoggedTunableNumber("PowerManager/BrownoutRecoveryAmpsPerSec", 50.0);

  private static final Alert conserveAlert =
      new Alert("Battery is low, drive power is being conserved.", AlertType.kInfo);
  private static final Alert criticalAlert =
      new Alert(
          "Battery is critically low, CHANGE BATTERY before the next match!", AlertType.kWarning);
  private static final Alert brownoutAlert =
      new Alert("Brownout detected, drive performance may be degraded.", AlertType.kWarning);

  private static PowerManager instance;

  public static PowerManager getInstance() {
    if (instance == null) instance = new PowerManager();
    return instance;
  }

  private PowerManager() {}

  // Accumulators for the current loop, reset in update()
  private double totalCurrentAccum = 0.0;
  private double driveCurrentAccum = 0.0;

  private final PowerManagerInputsAutoLogged inputs = new PowerManagerInputsAutoLogged();

  private double restVoltageEstimate = 12.6;
  private double driveBudget = Double.MAX_VALUE;
  private PowerTier tier = PowerTier.NORMAL;

  // Hold the brownout response for 2s after the rio recovers
  private final Debouncer brownoutDebouncer = new Debouncer(2.0, DebounceType.kFalling);
  private final Debouncer conserveDebouncer = new Debouncer(1.0, DebounceType.kBoth);
  private final Debouncer criticalDebouncer = new Debouncer(1.0, DebounceType.kBoth);

  /**
   * Reports the supply current drawn by one mechanism this loop. Call once per loop from the
   * subsystem's periodic(). Pass 0 when disconnected.
   *
   * @param key log name, e.g. "Hood" or "DriveModule0/Drive"
   * @param isDrive true only for drivetrain drive motors (they share the floating budget)
   * @param amps supply-side (battery-side) current in amps
   */
  public void reportCurrent(String key, boolean isDrive, double amps) {
    amps = Math.max(0.0, amps);
    totalCurrentAccum += amps;
    if (isDrive) driveCurrentAccum += amps;
    Logger.recordOutput("PowerManager/Currents/" + key, amps);
  }

  /** Runs the budget update. Call from Robot.robotPeriodic() AFTER CommandScheduler.run(). */
  public void update() {
    inputs.batteryVoltage = RobotController.getBatteryVoltage();
    inputs.brownedOut = RobotController.isBrownedOut();
    Logger.processInputs("PowerManager", inputs);

    double vBatt = inputs.batteryVoltage;
    double r0 = batteryResistanceOhms.get();
    double totalCurrent = totalCurrentAccum + overheadAmps.get();
    double driveCurrent = driveCurrentAccum;
    totalCurrentAccum = 0.0;
    driveCurrentAccum = 0.0;

    // IR-compensated rest voltage, low-pass filtered (tau = 2s)
    double instantRest = vBatt + totalCurrent * r0;
    double alpha = loopPeriodSecs / (2.0 + loopPeriodSecs);
    restVoltageEstimate += alpha * (instantRest - restVoltageEstimate);

    // Total current the battery can source without sagging below the brownout floor
    double batteryMaxCurrent = Math.max(0.0, (restVoltageEstimate - brownoutFloorVolts.get()) / r0);
    double budget = Math.min(batteryMaxCurrent * budgetHeadroom.get(), maxTotalBudgetAmps.get());

    // Drive gets whatever the mechanisms aren't using
    double calculatedDriveBudget = budget - (totalCurrent - driveCurrent);
    boolean brownedOut = brownoutDebouncer.calculate(inputs.brownedOut);
    if (!brownedOut) {
      driveBudget = calculatedDriveBudget;
    } else {
      // Asymmetric: drop instantly, recover slowly
      driveBudget =
          calculatedDriveBudget < driveBudget
              ? calculatedDriveBudget
              : Math.min(
                  calculatedDriveBudget,
                  driveBudget + brownoutRecoveryAmpsPerSec.get() * loopPeriodSecs);
    }
    driveBudget = Math.max(0.0, driveBudget);

    // Tier state machine
    boolean critical = criticalDebouncer.calculate(restVoltageEstimate < criticalRestVolts.get());
    boolean conserve = conserveDebouncer.calculate(restVoltageEstimate < conserveRestVolts.get());
    tier = critical ? PowerTier.CRITICAL : conserve ? PowerTier.CONSERVE : PowerTier.NORMAL;

    conserveAlert.set(tier == PowerTier.CONSERVE);
    criticalAlert.set(tier == PowerTier.CRITICAL);
    brownoutAlert.set(brownedOut);

    Logger.recordOutput("PowerManager/RestVoltage", restVoltageEstimate);
    Logger.recordOutput("PowerManager/TotalCurrentAmps", totalCurrent);
    Logger.recordOutput("PowerManager/DriveCurrentAmps", driveCurrent);
    Logger.recordOutput("PowerManager/BudgetAmps", budget);
    Logger.recordOutput("PowerManager/DriveBudgetAmps", driveBudget);
    Logger.recordOutput("PowerManager/BrownedOut", brownedOut);
    Logger.recordOutput("PowerManager/Tier", tier);
  }

  /**
   * Per-module drive supply current limit, quantized to reduce config churn on the CAN bus. Fixed
   * in autonomous, floats with the budget in teleop.
   */
  public double getDriveModuleSupplyLimitAmps() {
    double limit;
    if (DriverStation.isAutonomous()) {
      limit = driveAutoModuleAmps.get();
    } else {
      limit = MathUtil.clamp(driveBudget / 4.0, driveModuleMinAmps.get(), driveModuleMaxAmps.get());
      limit = Math.floor(limit / quantizationAmps) * quantizationAmps;
    }
    Logger.recordOutput("PowerManager/DriveModuleLimitAmps", limit);
    return limit;
  }

  public PowerTier getTier() {
    return tier;
  }

  /** True when mechanisms should shed comfort loads (flywheel idle, LED brightness, etc.). */
  public boolean shouldConserve() {
    return tier != PowerTier.NORMAL;
  }

  public boolean isCritical() {
    return tier == PowerTier.CRITICAL;
  }

  /** IR-compensated battery rest voltage estimate — the "is this battery actually low" signal. */
  public double getRestVoltageEstimate() {
    return restVoltageEstimate;
  }
}
