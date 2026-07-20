package frc.lib.rebuilt;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;

public class ShiftUtil {
  public enum ShiftEnum {
    TRANSITION,
    SHIFT1,
    SHIFT2,
    SHIFT3,
    SHIFT4,
    ENDGAME,
    AUTO,
    DISABLED;
  }

  public enum AutoWinOverrideState {
    UNKNOWN,
    WIN,
    LOSE
  }

  private static final String allianceWinOverrideKey = "Shift/Alliance Win Override";
  private static AutoWinOverrideState winOverrideState = AutoWinOverrideState.UNKNOWN;

  private static boolean validWinReportSignal = false;

  public record ShiftInfo(
      ShiftEnum currentShift,
      double elapsedTime,
      double remainingTime,
      double remainingFMSTime,
      double remainingInternalTime,
      boolean active) {}

  private static Timer shiftTimer = new Timer();
  private static final ShiftEnum[] shiftsEnums = ShiftEnum.values();

  private static final double[] shiftStartTimes = {0.0, 11.0, 36.0, 61.0, 86.0, 111.0};
  private static final double[] shiftEndTimes = {11.0, 36.0, 61.0, 86.0, 111.0, 141.0};

  public static final double autoEndTime = 20.0;
  public static final double teleopDuration = 140.0;
  private static double shiftTimerOffset = 0.0;
  private static final double timeResetThreshold = 2.0;
  private static final boolean[] activeSchedule = {true, true, false, true, false, true};
  private static final boolean[] inactiveSchedule = {true, false, true, false, true, true};

  private static Supplier<Optional<Boolean>> allianceWinOverride = () -> Optional.empty();

  public static void setWinAutoOverride(Supplier<Optional<Boolean>> override) {
    allianceWinOverride = (override != null) ? override : () -> Optional.empty();
    SmartDashboard.putBoolean(allianceWinOverrideKey, getAllianceWinOverride().orElse(false));
  }

  public static Optional<Boolean> getAllianceWinOverride() {
    try {
      var value =
          allianceWinOverride != null ? allianceWinOverride.get() : Optional.<Boolean>empty();
      return value != null ? value : Optional.empty();
    } catch (Exception exception) {
      DriverStation.reportWarning(
          "Failed to read alliance win override: " + exception.getMessage(), false);
      return Optional.empty();
    }
  }

  public static Alliance getFirstActiveAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    // Return override value
    var winOverride = getAllianceWinOverride();
    if (!winOverride.isEmpty()) {
      validWinReportSignal = true;
      return winOverride.get()
          ? (alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue)
          : (alliance == Alliance.Blue ? Alliance.Blue : Alliance.Red);
    }

    // Return FMS value
    String message = DriverStation.getGameSpecificMessage();
    if (message.length() > 0) {
      char character = message.charAt(0);
      if (character == 'R') {
        validWinReportSignal = true;
        return Alliance.Blue;
      } else if (character == 'B') {
        validWinReportSignal = true;
        return Alliance.Red;
      }
    }

    // Return default value
    validWinReportSignal = false;
    return alliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;
  }

  /** Starts the timer at the beginning of teleop. */
  public static void initialize() {
    setWinAutoOverride(
        () -> {
          switch (winOverrideState) {
            case WIN:
              return Optional.of(true);
            case LOSE:
              return Optional.of(false);
            case UNKNOWN:
            default:
              return Optional.empty();
          }
        });
    shiftTimer.restart();
    shiftTimerOffset = 0;
  }

  private static boolean[] getSchedule() {
    boolean[] currentSchedule;
    Alliance startAlliance = getFirstActiveAlliance();
    currentSchedule =
        startAlliance == DriverStation.getAlliance().orElse(Alliance.Blue)
            ? activeSchedule
            : inactiveSchedule;
    return currentSchedule;
  }

  private static ShiftInfo getShiftInfo(
      boolean[] currentSchedule, double[] shiftStartTimes, double[] shiftEndTimes) {
    double internalTime = shiftTimer.get();
    double currentTime = internalTime - shiftTimerOffset;
    double fieldTime = teleopDuration - DriverStation.getMatchTime();
    double stateTimeElapsed = shiftTimer.get();
    double stateTimeRemaining = 0.0;
    double fmsTimeElapsed = teleopDuration - DriverStation.getMatchTime();
    double internalStateTimeRemaining = shiftTimer.get();
    double internalTimeElapsed = 0.0;
    boolean active = false;
    ShiftEnum currentShift = ShiftEnum.DISABLED;

    if (DriverStation.isAutonomousEnabled()) {
      stateTimeElapsed = currentTime;
      stateTimeRemaining = autoEndTime - currentTime;
      active = true;
      currentShift = ShiftEnum.AUTO;
    } else if (DriverStation.isEnabled()) {
      int currentShiftIndex = -1;
      if (Math.abs(fieldTime - currentTime) >= timeResetThreshold
          && fieldTime <= 135
          && DriverStation.isFMSAttached()) {
        shiftTimerOffset += currentTime - fieldTime;
        currentTime = internalStateTimeRemaining + shiftTimerOffset;
      }
      for (int i = 0; i < shiftStartTimes.length; i++) {
        if (currentTime >= shiftStartTimes[i] && currentTime < shiftEndTimes[i]) {
          currentShiftIndex = i;
          break;
        }
      }
      if (currentShiftIndex < 0) {
        // After last shift, so assume endgame
        currentShiftIndex = shiftStartTimes.length - 1;
      }

      // Calculate elapsed and remaining time in the current shift, ignoring combined shifts
      stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex];
      stateTimeRemaining = shiftEndTimes[currentShiftIndex] - currentTime;

      // If the state is the same as the last shift, combine the elapsed time
      if (currentShiftIndex > 0) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex - 1]
            && validWinReportSignal) {
          stateTimeElapsed = currentTime - shiftStartTimes[currentShiftIndex - 1];
        }
      }

      // If the state is the same as the next shift, combine the remaining time
      if (currentShiftIndex < shiftEndTimes.length - 1) {
        if (currentSchedule[currentShiftIndex] == currentSchedule[currentShiftIndex + 1]
            && validWinReportSignal) {
          stateTimeRemaining = shiftEndTimes[currentShiftIndex + 1] - currentTime;
        }
      }

      active = currentSchedule[currentShiftIndex];
      currentShift = shiftsEnums[currentShiftIndex];
    }
    ShiftInfo shiftInfo =
        new ShiftInfo(
            currentShift,
            stateTimeElapsed,
            stateTimeRemaining,
            fmsTimeElapsed,
            internalTimeElapsed,
            active);
    return shiftInfo;
  }

  public static ShiftInfo getOfficialShiftInfo() {
    return getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes);
  }

  public static ShiftInfo getShiftedShiftInfo() {
    boolean[] shiftSchedule = getSchedule();
    // Starting active
    if (shiftSchedule[1] == true) {
      double[] shiftedShiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
      double[] shiftedShiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};
      return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
    }
    double[] shiftedShiftStartTimes = {0.0, 10.0, 35.0, 60.0, 85.0, 110.0};
    double[] shiftedShiftEndTimes = {10.0, 35.0, 60.0, 85.0, 110.0, 140.0};
    return getShiftInfo(shiftSchedule, shiftedShiftStartTimes, shiftedShiftEndTimes);
  }

  public static void publishShiftInfo() {
    SmartDashboard.putBoolean(allianceWinOverrideKey, getAllianceWinOverride().orElse(false));
    // boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    // Color winningAllianceColor =
    //     switch (getFirstActiveAlliance()) {
    //       case Blue:
    //         yield isRedAlliance ? Color.kLimeGreen : Color.kPurple;
    //       case Red:
    //         yield isRedAlliance ? Color.kRed : Color.kLimeGreen;
    //       default:
    //         yield Color.kGray;
    //     };
    // if (!validWinReportSignal) winningAllianceColor = Color.kGray;

    // Color activeColor =
    //     getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes).active()
    //         ? Color.kLimeGreen
    //         : Color.kPurple;
    // if (!validWinReportSignal) activeColor = Color.kGray;
    SmartDashboard.putNumber(
        "Shift/Time Left in Shift ",
        getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes).remainingTime());
    SmartDashboard.putNumber("DS Timer", DriverStation.getMatchTime());

    // Publish internal timer vs DriverStation match time for debugging and comparison
    double internalTimerSeconds = shiftTimer.get();
    double dsMatchTimeSeconds = DriverStation.getMatchTime();
    double teleopElapsedFromDS =
        Double.isNaN(dsMatchTimeSeconds) ? Double.NaN : teleopDuration - dsMatchTimeSeconds;
    double internalMinusDs =
        Double.isNaN(teleopElapsedFromDS) ? Double.NaN : internalTimerSeconds - teleopElapsedFromDS;

    SmartDashboard.putNumber("Shift/Internal Timer Seconds", internalTimerSeconds);
    SmartDashboard.putNumber(
        "Shift/DS Match Time Seconds",
        Double.isNaN(dsMatchTimeSeconds) ? -1.0 : dsMatchTimeSeconds);
    SmartDashboard.putNumber(
        "Shift/Teleop Elapsed From DS Seconds",
        Double.isNaN(teleopElapsedFromDS) ? -1.0 : teleopElapsedFromDS);
    SmartDashboard.putNumber(
        "Shift/InternalMinusDS Seconds", Double.isNaN(internalMinusDs) ? 0.0 : internalMinusDs);
    SmartDashboard.putString(
        "Shift/Shift ",
        getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes).currentShift().name());
    SmartDashboard.putBoolean(
        "Shift/Is Our Shift ",
        getShiftInfo(getSchedule(), shiftStartTimes, shiftEndTimes).active());
    SmartDashboard.putBoolean("Shift/Did We Win Auto", validWinReportSignal);
  }

  public static void bindWinOverride(Trigger winTrigger, Trigger loseTrigger) {
    winTrigger.onTrue(
        Commands.runOnce(
                () -> {
                  winOverrideState = AutoWinOverrideState.WIN;
                })
            .ignoringDisable(true));
    loseTrigger.onTrue(
        Commands.runOnce(
                () -> {
                  winOverrideState = AutoWinOverrideState.LOSE;
                })
            .ignoringDisable(true));
  }
}
