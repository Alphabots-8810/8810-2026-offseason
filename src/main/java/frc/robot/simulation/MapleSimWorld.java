package frc.robot.simulation;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;

public final class MapleSimWorld {
  private static final boolean MAPLE_SIM_EFFICIENCY_MODE = true;
  private static final Pose2d INITIAL_POSE = new Pose2d(1.5, 4.0, Rotation2d.kZero);

  private static MapleSimWorld instance;

  private final Arena2026Rebuilt arena;
  private final SwerveDriveSimulation driveSimulation;
  private final IntakeSimulation fuelIntake;

  private MapleSimWorld() {
    arena = new Arena2026Rebuilt();
    arena.setEfficiencyMode(MAPLE_SIM_EFFICIENCY_MODE);
    SimulatedArena.overrideInstance(arena);

    driveSimulation = new SwerveDriveSimulation(createDriveConfig(), INITIAL_POSE);
    arena.addDriveTrainSimulation(driveSimulation);

    fuelIntake =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel", driveSimulation, Meters.of(0.7), Meters.of(0.25), IntakeSide.BACK, 20);
    resetField();
  }

  public static synchronized MapleSimWorld getInstance() {
    if (Constants.currentMode != Constants.Mode.SIM) {
      throw new IllegalStateException("MapleSimWorld can only be created in SIM mode.");
    }

    if (instance == null) {
      instance = new MapleSimWorld();
    }
    return instance;
  }

  public static void setFuelIntakeRunning(boolean running) {
    if (Constants.currentMode != Constants.Mode.SIM) {
      return;
    }

    MapleSimWorld world = getInstance();
    if (running) {
      world.fuelIntake.startIntake();
    } else {
      world.fuelIntake.stopIntake();
    }
  }

  public static void setRobotPose(Pose2d pose) {
    if (Constants.currentMode == Constants.Mode.SIM && instance != null) {
      instance.setRobotPoseInternal(pose);
    }
  }

  public SwerveDriveSimulation getDriveSimulation() {
    return driveSimulation;
  }

  public Pose2d getRobotPose() {
    Pose2d pose = driveSimulation.getSimulatedDriveTrainPose();
    return isFinite(pose) ? pose : INITIAL_POSE;
  }

  public void resetField() {
    arena.resetFieldForAuto();
    fuelIntake.stopIntake();
    fuelIntake.setGamePiecesCount(0);
    setRobotPoseInternal(INITIAL_POSE);
  }

  public void simulationPeriodic() {
    arena.simulationPeriodic();

    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    boolean recoveredPose = !isFinite(robotPose);
    if (recoveredPose) {
      setRobotPoseInternal(INITIAL_POSE);
      robotPose = INITIAL_POSE;
    }

    Logger.recordOutput("FieldSimulation/Robot", robotPose);
    Logger.recordOutput("FieldSimulation/RecoveredNaNPose", recoveredPose);
    Logger.recordOutput("FieldSimulation/Fuel", arena.getGamePiecesArrayByType("Fuel"));
    Logger.recordOutput("FieldSimulation/FuelInIntake", fuelIntake.getGamePiecesAmount());
    Logger.recordOutput("FieldSimulation/FuelIntakeRunning", fuelIntake.isRunning());
  }

  private void setRobotPoseInternal(Pose2d pose) {
    Pose2d safePose = isFinite(pose) ? pose : INITIAL_POSE;
    driveSimulation.setSimulationWorldPose(safePose);
    driveSimulation.getGyroSimulation().setRotation(safePose.getRotation());
  }

  private static DriveTrainSimulationConfig createDriveConfig() {
    return DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(74.088))
        .withBumperSize(Meters.of(0.85), Meters.of(0.85))
        .withCustomModuleTranslations(Drive.getModuleTranslations())
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(createModuleConfig());
  }

  private static SwerveModuleSimulationConfig createModuleConfig() {
    return new SwerveModuleSimulationConfig(
        DCMotor.getKrakenX60Foc(1),
        DCMotor.getKrakenX60Foc(1),
        TunerConstants.FrontLeft.DriveMotorGearRatio,
        TunerConstants.FrontLeft.SteerMotorGearRatio,
        Volts.of(0.2),
        Volts.of(0.2),
        Meters.of(TunerConstants.FrontLeft.WheelRadius),
        KilogramSquareMeters.of(0.03),
        1.2);
  }

  private static boolean isFinite(Pose2d pose) {
    return pose != null
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }
}
