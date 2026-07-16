package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.AutopilotTrenchCommand.AutopilotTrenchCommand;
import frc.robot.commands.IntakeDeployZeroCommand.IntakeDeployZeroCommand;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Dashboard-configurable "building block" autonomous.
 *
 * <p>Scans {@code deploy/choreo} for trajectories named {@code <Side><Section><Variant>.traj} (e.g.
 * {@code Left1InnerSwap.traj} = side Left, section 1, variant InnerSwap) and publishes four
 * choosers: side (Left/Right) plus one variant chooser per section. The auto is pre-built while the
 * robot sits disabled (rebuilt whenever a chooser changes, see {@link #updatePrebuild()}), so
 * choosers can be changed on Elastic right up to the match and autonomous still starts instantly —
 * no path loading or command construction happens at auto init:
 *
 * <ol>
 *   <li>Outward-zero the intake deploy, then run Section 1 while deploying and intaking
 *   <li>Timed shooting
 *   <li>Autopilot trench pass + Section 2 path, intaking from the moment the shot ends
 *   <li>Timed shooting
 *   <li>Autopilot trench pass + Section 3 path, intaking from the moment the shot ends
 * </ol>
 *
 * <p>Selecting {@code None} for a section skips that path (the shots/trench passes around it still
 * run). A selected variant that has no matching file for the chosen side aborts to a safe no-op
 * with a Driver Station error instead of crashing.
 */
public class BlockAutoBuilder {
  private static final String NONE_OPTION = "None";
  /** Chooser label for a trajectory with no variant suffix (e.g. plain "Left3.traj"). */
  private static final String BASE_VARIANT = "(base)";

  private static final int NUM_SECTIONS = 3;
  private static final Pattern TRAJ_PATTERN = Pattern.compile("^(Left|Right)([1-3])(.*)\\.traj$");

  /**
   * Hand-registered variants, merged with the ones discovered from deploy/choreo. Add a variant
   * here to force it onto a section's chooser even when no matching .traj is deployed yet (it
   * errors cleanly at auto start if the file is still missing). Key = section 1-3.
   */
  private static final Map<Integer, List<String>> EXTRA_VARIANTS =
      Map.of(
          1, List.of(),
          2, List.of(),
          3, List.of());

  private final LoggedDashboardChooser<String> sideChooser;
  private final List<LoggedDashboardChooser<String>> sectionChoosers = new ArrayList<>();

  /** Auto pre-built while disabled, handed out by {@link #buildCommand()}'s deferred supplier. */
  private Command prebuiltAuto = null;
  /** Chooser values (side, then sections 1-3) the prebuilt auto was built from; null = none. */
  private String[] prebuiltSelection = null;

  public BlockAutoBuilder() {
    sideChooser = new LoggedDashboardChooser<>("BlockAuto/Side");
    sideChooser.addDefaultOption("Left", "Left");
    sideChooser.addOption("Right", "Right");

    Map<Integer, Set<String>> variantsBySection = scanVariants();
    for (int section = 1; section <= NUM_SECTIONS; section++) {
      LoggedDashboardChooser<String> chooser =
          new LoggedDashboardChooser<>("BlockAuto/Path" + section);
      Set<String> variants = new TreeSet<>(variantsBySection.getOrDefault(section, Set.of()));
      variants.addAll(EXTRA_VARIANTS.getOrDefault(section, List.of()));
      boolean first = true;
      for (String variant : variants) {
        if (first) {
          chooser.addDefaultOption(variant, variant);
          first = false;
        } else {
          chooser.addOption(variant, variant);
        }
      }
      if (first) {
        chooser.addDefaultOption(NONE_OPTION, NONE_OPTION);
      } else {
        chooser.addOption(NONE_OPTION, NONE_OPTION);
      }
      sectionChoosers.add(chooser);
    }
  }

  /**
   * Variant suffixes found in deploy/choreo, grouped by section number. The union across both sides
   * is offered, so a variant that only exists for one side is still selectable (and errors cleanly
   * if picked for the side that lacks it).
   */
  private static Map<Integer, Set<String>> scanVariants() {
    Map<Integer, Set<String>> result = new HashMap<>();
    File[] files = new File(Filesystem.getDeployDirectory(), "choreo").listFiles();
    if (files == null) {
      return result;
    }
    for (File file : files) {
      Matcher matcher = TRAJ_PATTERN.matcher(file.getName());
      if (matcher.matches()) {
        String variant = matcher.group(3).isEmpty() ? BASE_VARIANT : matcher.group(3);
        result
            .computeIfAbsent(Integer.parseInt(matcher.group(2)), k -> new TreeSet<>())
            .add(variant);
      }
    }
    return result;
  }

  /**
   * The command to register on the main auto chooser. Deferred, but the supplier normally just
   * hands out the auto pre-built while disabled — building at auto init only happens as a fallback
   * if the choosers changed in the same cycle autonomous started.
   */
  public Command buildCommand() {
    return Commands.defer(
        this::takePrebuiltOrBuild,
        Set.<Subsystem>of(
            Drive.mInstance,
            Drum.mInstance,
            Feeder.mInstance,
            Hood.mInstance,
            Indexer.mInstance,
            IntakeDeploy.mInstance,
            IntakeRoller.mInstance));
  }

  /**
   * Call every loop while the robot is disabled: rebuilds the cached auto only when the
   * side/variant selection actually changed, so all Choreo path loading and command construction
   * cost is paid before the match instead of at auto init. The steady-state check is
   * allocation-free (plain string compares against the cached selection) — per-loop garbage on the
   * RIO shows up as GC pauses and CANivore timestamp-desync warnings. Does nothing while enabled —
   * a mid-match rebuild would fight the running auto for path/command state.
   */
  public void updatePrebuild() {
    if (!DriverStation.isDisabled() || selectionMatchesPrebuilt()) {
      return;
    }
    // Capture the selection before building so a chooser edit mid-build just looks like another
    // change next cycle.
    String[] selection = new String[NUM_SECTIONS + 1];
    selection[0] = sideChooser.get();
    for (int i = 0; i < NUM_SECTIONS; i++) {
      selection[i + 1] = sectionChoosers.get(i).get();
    }
    prebuiltAuto = buildSelectedAuto();
    prebuiltSelection = selection;
  }

  /** True when a prebuilt auto exists and the choosers still read the values it was built from. */
  private boolean selectionMatchesPrebuilt() {
    if (prebuiltSelection == null) {
      return false;
    }
    if (!Objects.equals(prebuiltSelection[0], sideChooser.get())) {
      return false;
    }
    for (int i = 0; i < NUM_SECTIONS; i++) {
      if (!Objects.equals(prebuiltSelection[i + 1], sectionChoosers.get(i).get())) {
        return false;
      }
    }
    return true;
  }

  /**
   * Deferred supplier: the prebuilt auto when it matches the current selection, otherwise a fresh
   * build (first enable after boot without a disabled cycle, or a chooser edit racing auto init).
   * The cache is cleared either way — DeferredCommand registers whatever it runs as composed, so an
   * instance can only ever be handed out once; the next disabled cycle pre-builds a fresh one.
   */
  private Command takePrebuiltOrBuild() {
    Command auto = selectionMatchesPrebuilt() ? prebuiltAuto : buildSelectedAuto();
    prebuiltAuto = null;
    prebuiltSelection = null;
    return auto;
  }

  /** Pre-built NT keys for the selection table — no per-loop string concatenation. */
  private static final String[] TABLE_KEYS = new String[NUM_SECTIONS];

  static {
    for (int i = 0; i < NUM_SECTIONS; i++) {
      TABLE_KEYS[i] = "BlockAuto/Table/Path" + (i + 1);
    }
  }

  /** Side + variants last written to the table, so unchanged selections publish nothing. */
  private String publishedSide = null;

  private final String[] publishedVariants = new String[NUM_SECTIONS];

  /**
   * Publishes only the three selected trajectory names for a compact dashboard preview. Runs every
   * loop, so it only builds and writes a display string when that selection actually changed —
   * steady-state cost is a few string compares, no allocation.
   */
  public void publishSelection() {
    String side = sideChooser.get();
    boolean sideChanged = !Objects.equals(side, publishedSide);
    for (int i = 0; i < NUM_SECTIONS; i++) {
      String variant = sectionChoosers.get(i).get();
      if (!sideChanged && Objects.equals(variant, publishedVariants[i])) {
        continue;
      }
      publishedVariants[i] = variant;
      String display =
          variant == null || NONE_OPTION.equals(variant)
              ? "None"
              : resolveTrajName(side, i + 1, variant);
      SmartDashboard.putString(TABLE_KEYS[i], display);
    }
    publishedSide = side;
  }

  /** Full trajectory base name for a side + section + chooser variant. */
  private static String resolveTrajName(String side, int section, String variant) {
    return side + section + (BASE_VARIANT.equals(variant) ? "" : variant);
  }

  private Command buildSelectedAuto() {
    String side = sideChooser.get();
    String[] pathNames = new String[NUM_SECTIONS];
    for (int i = 0; i < NUM_SECTIONS; i++) {
      String variant = sectionChoosers.get(i).get();
      pathNames[i] = NONE_OPTION.equals(variant) ? null : resolveTrajName(side, i + 1, variant);
    }
    DriverStation.reportWarning(
        "BlockAuto: side="
            + side
            + " paths="
            + pathNames[0]
            + ", "
            + pathNames[1]
            + ", "
            + pathNames[2],
        false);

    try {
      List<Command> steps = new ArrayList<>();
      // The drum stays off at auto start: the first path's "Drum_stow" event marker cuts the
      // intake and starts the drum ramping to StowVelocity partway through the path (see
      // AutoCommands.firstPathWithIntake), and Shooting.end() leaves StowVelocity after every shot.
      if (pathNames[0] != null) {
        steps.add(AutoCommands.firstPathWithIntake(pathNames[0]));
      } else {
        // No section-1 path means no "IntakeDeploy" marker ever fires, so zero the intake deploy
        // here — otherwise the boot encoder value (deployed) makes every later intake deploy
        // setpoint a no-op and the intake never comes down all auto.
        steps.add(
            new IntakeDeployZeroCommand(IntakeDeployZeroCommand.Direction.INWARD)
                .withTimeout(AutoCommandsConstants.INWARD_ZERO_TIMEOUT_SEC));
      }
      // Sections 2/3: the intake starts the moment the shot ends and runs through the trench
      // pass and the path, so fuel in and around the trench is collected too.
      for (int section = 1; section < NUM_SECTIONS; section++) {
        steps.add(AutoCommands.timedShoot(section == 1 ? 3 : 2));
        if (pathNames[section] != null) {
          steps.add(AutoCommands.trenchThenPathWithIntake(pathNames[section]));
        } else {
          steps.add(new AutopilotTrenchCommand());
        }
      }
      return Commands.sequence(steps.toArray(Command[]::new));
    } catch (RuntimeException e) {
      // Missing .traj for the chosen side/variant: never crash or run a partial auto.
      DriverStation.reportError("BlockAuto failed to load: " + e.getMessage(), false);
      return Commands.print("BlockAuto unavailable: missing Choreo trajectory");
    }
  }
}
