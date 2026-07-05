// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

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
import frc.robot.commands.AutoTrenchCommand.AutoTrenchCommand;
import frc.robot.commands.AutoalignIntakeCommand.AutoalignIntake;
import frc.robot.commands.DriveCommands.CornerPivotCommand;
import frc.robot.commands.DriveCommands.CornerPivotCommand.PivotCorner;
import frc.robot.commands.DriveCommands.DriveCommands;
import frc.robot.commands.HoodZeroCommand.HoodZeroCommand;
import frc.robot.commands.IntakeCommand.IntakeCommand;
import frc.robot.commands.IntakeDeployOutwardZeroCommand.IntakeDeployOutwardZeroCommand;
import frc.robot.commands.ManualCommand.Manual;
import frc.robot.commands.ShootingCommand.Shooting;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.LoggedTunableNumber;
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

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        Drive.mInstance = drive;
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

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
    controller
        .povUp()
        .whileTrue(
            new CornerPivotCommand(drive, PivotCorner.FRONT_LEFT, () -> -controller.getRightX()));
    controller
        .povLeft()
        .whileTrue(Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0, 2, 0)), drive));
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
    controller.rightBumper().whileTrue(new Shooting());
    controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  Indexer.mInstance.setV(-3);
                  Feeder.mInstance.setV(-3);
                }));
    controller
        .rightTrigger()
        .onFalse(
            new InstantCommand(
                () -> {
                  Indexer.mInstance.setV(0);
                  Feeder.mInstance.setV(0);
                }));

    controller.y().onTrue(new HoodZeroCommand().alongWith(new IntakeDeployOutwardZeroCommand()));
    controller
        .rightStick()
        .onTrue(
            new AutoalignIntake(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));
    controller.leftTrigger(0.1).whileTrue(new IntakeCommand());
    controller
        .leftBumper()
        .whileTrue(
            new AutoalignIntake(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()));
    controller.a().whileTrue(new AutoTrenchCommand(() -> controller.getLeftY()));
    // controller.rightTrigger().whileTrue(new Shooting());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
