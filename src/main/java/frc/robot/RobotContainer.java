package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoCommands.AutoCommands;
import frc.robot.commands.AutoCommands.BlockAutoBuilder;
import frc.robot.commands.AutoTrenchCommand.AutoTrenchCommand;
import frc.robot.commands.AutoalignIntakeCommand.AutoalignIntake;
import frc.robot.commands.DriveCommands.DriveCommands;
import frc.robot.commands.FerryCommand.Ferry;
import frc.robot.commands.HoodZeroCommand.HoodZeroCommand;
import frc.robot.commands.IntakeCommand.IntakeCommand;
import frc.robot.commands.IntakeDeployZeroCommand.IntakeDeployZeroCommand;
import frc.robot.commands.ManualCommand.Manual;
import frc.robot.commands.ShootingCommand.Shooting;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.FuelSimulation;
import frc.robot.simulation.MapleSimArena;
import frc.robot.subsystems.Drum.Drum;
import frc.robot.subsystems.FeedPath.FeedPath;
import frc.robot.subsystems.Feeder.Feeder;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.IntakeRoller.IntakeRoller;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Cameras;
import frc.robot.util.LoggedTunableNumber;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Touching each singleton here forces its class to initialize at boot, which runs the
  // constructor and flashes the motor config (gear ratio, PID, etc). Without an eager reference
  // these only initialize on first button press, so the config is never applied at deploy.
  private final Drum drum = Drum.mInstance;
  private final FeedPath feedPath = FeedPath.mInstance;
  private final Feeder feeder = Feeder.mInstance;
  private final Hood hood = Hood.mInstance;
  private final Indexer indexer = Indexer.mInstance;
  private final IntakeDeploy intakeDeploy = IntakeDeploy.mInstance;
  private final IntakeRoller intakeRoller = IntakeRoller.mInstance;
  private final LoggedTunableNumber retract = new LoggedTunableNumber("retract", 25);

  // Created after the drive in the constructor because its cameras read the drive pose
  private Cameras cameras = Cameras.mInstance;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final BlockAutoBuilder blockAutoBuilder = new BlockAutoBuilder();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        Drive.mInstance = drive;

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot: maple-sim arena + AdvantageKit drive IO (official hardware-abstraction path)
        var driveSimulation = MapleSimArena.getInstance().getDriveSimulation();
        FuelSimulation.configure(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]));
        Drive.mInstance = drive;
        drive.setPose(MapleSimArena.getInstance().getRobotPose());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        Drive.mInstance = drive;
        break;
    }
    Drive.mInstance = drive;

    // Initialize the camera subsystem after Drive.mInstance is set, since its
    // periodic reads the drive pose
    cameras = Cameras.mInstance;

    // Choreo event-marker bindings. Must be registered before any path is loaded: PathPlanner
    // resolves a marker's command from NamedCommands at .traj parse time. On the Left1*/Right1*
    // trajectories: "IntakeZero" at t=0 inward-zeroes the intake deploy while the robot launches
    // (it starts the match seated on the stowed hard stop, so the spike is quick even while
    // driving); "IntakeDeploy" at ~0.55 s deploys and runs the intake, canceled (roller/indexer
    // stopped) when the path ends. Event commands share the EventScheduler, where a later command
    // cancels an earlier one with overlapping requirements — so a zero that never sees its current
    // spike is simply superseded by the deploy (encoder not reset, intake stays stowed) instead of
    // needing a timeout.
    new EventTrigger("IntakeZero")
        .onTrue(new IntakeDeployZeroCommand(IntakeDeployZeroCommand.Direction.INWARD));
    new EventTrigger("IntakeDeploy").onTrue(new IntakeCommand());
    new EventTrigger("IntakeZeroOut")
        .onTrue(new IntakeDeployZeroCommand(IntakeDeployZeroCommand.Direction.OUTWARD));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption("Left Three-Piece", AutoCommands.leftThreePieceAuto());
    autoChooser.addOption("PID Path Only", AutoCommands.pidTestPathAuto());

    // Building-block auto: side + per-section path variants selected on the BlockAuto/* choosers,
    // assembled when autonomous starts.
    autoChooser.addOption("Block Auto (dashboard)", blockAutoBuilder.buildCommand());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller.povUp().whileTrue(new AutoTrenchCommand(() -> controller.getLeftY()));
    controller
        .povLeft()
        .onTrue(new IntakeDeployZeroCommand(IntakeDeployZeroCommand.Direction.INWARD));
    controller
        .povDown()
        .whileTrue(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-2, 0, 0)), drive));
    controller
        .povRight()
        .whileTrue(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, -2, 0)), drive));

    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      // Zero heading toward the opposing alliance wall. On red the field is
                      // flipped, so "forward" is 180 deg from the blue origin frame.
                      boolean isFlipped =
                          DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red;
                      drive.setPose(
                          new Pose2d(
                              drive.getPose().getTranslation(),
                              isFlipped ? Rotation2d.kPi : Rotation2d.kZero));
                    },
                    drive)
                .ignoringDisable(true));

    // controller
    //     .a()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               IntakeDeploy.mInstance.setPositionCentimeter(retract.getAsDouble(), 50, 1000,
    // 0);
    //               IntakeRoller.mInstance.setVelocityRotPerSec(40);
    //             },
    //             IntakeDeploy.mInstance,
    //             IntakeRoller.mInstance));
    // controller
    //     .a()
    //     .onFalse(
    //         new InstantCommand(
    //             () -> {
    //               IntakeDeploy.mInstance.setPositionCentimeter(retract.getAsDouble(), 50, 1000,
    // 0);
    //               IntakeRoller.mInstance.setV(0);
    //             },
    //             IntakeDeploy.mInstance,
    //             IntakeRoller.mInstance));
    controller.x().whileTrue(new Manual());
    controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  Indexer.mInstance.setV(-3);
                  Feeder.mInstance.setV(-3);
                  IntakeRoller.mInstance.setV(-3);
                }));
    controller
        .rightTrigger()
        .onFalse(
            new InstantCommand(
                () -> {
                  Indexer.mInstance.setV(0);
                  Feeder.mInstance.setV(0);
                  IntakeRoller.mInstance.setV(0);
                }));

    controller
        .y()
        .onTrue(
            new HoodZeroCommand()
                .alongWith(new IntakeDeployZeroCommand(IntakeDeployZeroCommand.Direction.OUTWARD)));
    controller.rightBumper().whileTrue(shootOrFerryCommand());
    controller.leftTrigger(0.1).whileTrue(new IntakeCommand());
    controller
        .leftBumper()
        .whileTrue(
            new AutoalignIntake(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));
    controller
        .a()
        .whileTrue(new Shooting(true, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    // Sim-only: Start button clears every ball on the field and resets the robot pose / intake
    // stock. Ignored entirely on a real robot so the binding can stay in the code.
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (Constants.currentMode == Constants.Mode.SIM
                          && MapleSimArena.getInstance() != null) {
                        MapleSimArena.getInstance().resetField();
                      }
                    })
                .ignoringDisable(true));
  }

  /** True when the robot is on its alliance side of the HUB. */
  private boolean isInAllianceArea() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    return FieldLayout.isPoseInAllianceArea(isRed, drive.getPose());
  }

  /** Alliance side of the HUB: shoot at the HUB; neutral side: ferry. */
  private Command shootOrFerryCommand() {
    return Commands.defer(
        () ->
            isInAllianceArea()
                ? new Shooting(false, () -> -controller.getLeftY(), () -> -controller.getLeftX())
                : new Ferry(),
        Set.of(drive, drum, feeder, hood, indexer, intakeDeploy, intakeRoller));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Dashboard outputs refreshed every loop (block-auto selection table, etc.). */
  public void updateDashboardOutputs() {
    blockAutoBuilder.publishSelection();
  }
}
