package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.AutopilotTrenchCommand.AutopilotTrenchCommand;
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
 * choosers: side (Left/Right) plus one variant chooser per section. The auto is assembled when
 * autonomous starts, so choosers can be changed on Elastic right up to the match:
 *
 * <ol>
 *   <li>Section 1 path (reset odometry to its start, intake 1 s in)
 *   <li>Timed shooting
 *   <li>Autopilot trench pass
 *   <li>Section 2 path (with intake)
 *   <li>Timed shooting
 *   <li>Autopilot trench pass
 *   <li>Section 3 path (with intake)
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
  // Base names (no .traj) of every trajectory deployed at boot; used for the dashboard table's
  // OK/MISSING check without touching the filesystem every loop.
  private final Set<String> availableTrajectories = new TreeSet<>();

  public BlockAutoBuilder() {
    sideChooser = new LoggedDashboardChooser<>("BlockAuto/Side");
    sideChooser.addDefaultOption("Left", "Left");
    sideChooser.addOption("Right", "Right");

    File[] trajFiles = new File(Filesystem.getDeployDirectory(), "choreo").listFiles();
    if (trajFiles != null) {
      for (File file : trajFiles) {
        if (file.getName().endsWith(".traj")) {
          availableTrajectories.add(file.getName().substring(0, file.getName().length() - 5));
        }
      }
    }

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
   * The command to register on the main auto chooser. Deferred: the selected side/variants are read
   * and the sequence assembled when autonomous actually starts.
   */
  public Command buildCommand() {
    return Commands.defer(
        this::buildSelectedAuto,
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
   * Publishes a readable preview of the currently selected auto to the dashboard. Call every loop
   * (cheap: string writes only, NT dedupes unchanged values). Shown under {@code
   * SmartDashboard/BlockAuto/}: per-section resolved trajectory names with an OK/MISSING check, and
   * the full step-by-step sequence as a text table.
   */
  public void publishSelection() {
    String side = sideChooser.get();
    List<String> lines = new ArrayList<>();
    boolean allOk = true;

    for (int i = 0; i < NUM_SECTIONS; i++) {
      String variant = sectionChoosers.get(i).get();
      int section = i + 1;
      String status;
      String display;
      if (variant == null || NONE_OPTION.equals(variant)) {
        display = "(skipped)";
        status = "skipped";
      } else {
        String name = resolveTrajName(side, section, variant);
        boolean exists = availableTrajectories.contains(name);
        allOk &= exists;
        display = name;
        status = exists ? "OK" : "MISSING";
      }
      SmartDashboard.putString("BlockAuto/Table/Path" + section, display + "  [" + status + "]");

      if (!"skipped".equals(status)) {
        String extras = section == 1 ? " + reset pose + intake" : " + intake";
        lines.add((lines.size() + 1) + ". Path " + display + " [" + status + "]" + extras);
      }
      if (i < NUM_SECTIONS - 1) {
        lines.add(
            (lines.size() + 1) + ". Shoot (" + AutoCommandsConstants.SHOOTING_DURATION_SEC + "s)");
        lines.add((lines.size() + 1) + ". Trench pass (Autopilot)");
      }
    }

    SmartDashboard.putStringArray("BlockAuto/Table/Sequence", lines.toArray(String[]::new));
    SmartDashboard.putString("BlockAuto/Table/SequenceText", String.join("\n", lines));
    SmartDashboard.putBoolean("BlockAuto/Table/AllTrajectoriesFound", allOk);
    SmartDashboard.putStringArray(
        "BlockAuto/Table/DeployedTrajectories", availableTrajectories.toArray(String[]::new));
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
      if (pathNames[0] != null) {
        steps.add(AutoCommands.firstPathWithIntake(pathNames[0]));
      }
      steps.add(AutoCommands.timedShoot());
      steps.add(new AutopilotTrenchCommand());
      if (pathNames[1] != null) {
        steps.add(AutoCommands.pathWithIntake(pathNames[1]));
      }
      steps.add(AutoCommands.timedShoot());
      steps.add(new AutopilotTrenchCommand());
      if (pathNames[2] != null) {
        steps.add(AutoCommands.pathWithIntake(pathNames[2]));
      }
      return Commands.sequence(steps.toArray(Command[]::new));
    } catch (RuntimeException e) {
      // Missing .traj for the chosen side/variant: never crash or run a partial auto.
      DriverStation.reportError("BlockAuto failed to load: " + e.getMessage(), false);
      return Commands.print("BlockAuto unavailable: missing Choreo trajectory");
    }
  }
}
