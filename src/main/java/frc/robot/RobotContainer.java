package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.subsystems.vision.Vision;
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

  // Created after the drive in the constructor because its camera reads the drive pose
  private Vision vision = Vision.mInstance;
  private Vision vision2 = Vision.mInstance2;

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

    // Initialize the vision subsystem after Drive.mInstance is set, since its
    // periodic reads the drive pose
    vision = Vision.mInstance;
    vision2 = Vision.mInstance2;

    // No EventTrigger bindings for Choreo event markers: EventTrigger commands are scheduled on
    // the main scheduler, so their subsystem requirements conflict with the deferred block auto
    // (which holds every subsystem) and cancel the entire auto the moment a marker fires. The
    // "IntakeZeroOut" work now runs inline before the first path in
    // AutoCommands.firstPathWithIntake; any marker still present in a .traj simply fires into an
    // unbound trigger.

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Commented out 2026-07-11: leftThreePieceAuto's Choreo trajectories no longer exist — see
    // the commented-out method in AutoCommands.
    // autoChooser.addOption("Left Three-Piece", AutoCommands.leftThreePieceAuto());
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
    // Pre-build the block auto while disabled so auto init pays no path-loading cost.
    blockAutoBuilder.updatePrebuild();
  }
}
