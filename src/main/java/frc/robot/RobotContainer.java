package frc.robot;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CanBus;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIODev;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import frc.robot.subsystems.sensors.NoteSensor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;
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
  private final Vision vision;
  private final Wrist wrist;
  private final Shooter shooter;
  private final RollerSystem intake;
  private final RollerSystem feederLower;
  private final RollerSystem feederUpper;
  private final NoteSensor noteSensor;

  // Controllers
  private static final CommandXboxController driverController = new CommandXboxController(0);
  private static final Alert driverControllerDisconnectedAlert =
      new Alert("Xbox controller disconnected.", AlertType.kError);
  private static final CommandPS4Controller operatorController = new CommandPS4Controller(1);
  private static final Alert OperatorControllerDisconnectedAlert =
      new Alert("PS4 controller disconnected.", AlertType.kError);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        switch (Constants.getRobot()) {
          case COMPBOT:
            drive =
                new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIODev(DriveConstants.moduleConfigsComp[0]),
                    new ModuleIODev(DriveConstants.moduleConfigsComp[1]),
                    new ModuleIODev(DriveConstants.moduleConfigsComp[2]),
                    new ModuleIODev(DriveConstants.moduleConfigsComp[3]));
            break;

          default:
            drive =
                new Drive(
                    new GyroIOPigeon2(),
                    new ModuleIODev(DriveConstants.moduleConfigsDev[0]),
                    new ModuleIODev(DriveConstants.moduleConfigsDev[1]),
                    new ModuleIODev(DriveConstants.moduleConfigsDev[2]),
                    new ModuleIODev(DriveConstants.moduleConfigsDev[3]));
            break;
        }

        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                drive::getFieldVelocity,
                new VisionIOLimelight("Intake", drive::getRotation));

        wrist = new Wrist(new WristIOTalonFX());

        shooter = new Shooter();

        intake =
            new RollerSystem(
                "Intake",
                new RollerSystemIOTalonFX(
                    17,
                    CanBus.rio,
                    40,
                    InvertedValue.Clockwise_Positive,
                    NeutralModeValue.Coast,
                    1));

        feederLower =
            new RollerSystem(
                "FeederLower",
                new RollerSystemIOTalonFX(
                    18,
                    CanBus.rio,
                    40,
                    InvertedValue.Clockwise_Positive,
                    NeutralModeValue.Brake,
                    1));
        feederUpper =
            new RollerSystem(
                "FeederUpper",
                new RollerSystemIOTalonFX(
                    19,
                    CanBus.rio,
                    40,
                    InvertedValue.Clockwise_Positive,
                    NeutralModeValue.Brake,
                    1));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                drive::getFieldVelocity,
                new VisionIO() {});

        wrist = new Wrist(new WristIOSim());

        shooter = new Shooter();

        intake =
            new RollerSystem("Intake", new RollerSystemIOSim(DCMotor.getFalcon500Foc(1), 1, .3));

        feederLower =
            new RollerSystem(
                "FeederLower", new RollerSystemIOSim(DCMotor.getFalcon500Foc(1), 1, .074));
        feederUpper =
            new RollerSystem(
                "FeederUpper", new RollerSystemIOSim(DCMotor.getFalcon500Foc(1), 1, .074));
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                drive::getFieldVelocity,
                new VisionIO() {});

        wrist = new Wrist(new WristIO() {});

        shooter = new Shooter();

        intake = new RollerSystem("Intake", new RollerSystemIO() {});

        feederLower = new RollerSystem("FeederLower", new RollerSystemIO() {});
        feederUpper = new RollerSystem("FeederUpper", new RollerSystemIO() {});
        break;
    }

    intake.setPID(new SlotConfigs().withKP(5).withKD(0).withKS(0));

    feederLower.setPID(new SlotConfigs().withKP(10).withKD(0).withKS(0));

    feederUpper.setPID(new SlotConfigs().withKP(15).withKD(0).withKS(0));

    noteSensor = new NoteSensor();

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

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController
        .leftTrigger()
        .onTrue(
            // Commands.parallel(
            IntakeCommands.manualIntake(
                intake, feederLower, feederUpper, wrist, () -> noteSensor.isHaveNote()));
    // noteSensor.automaticSimulatedNote(3))
    // .onlyIf(
    //     () ->
    //         Constants.getMode() == Mode.SIM && noteSensor.getAutomaticNoteSim().get()));
    driverController.rightTrigger().onTrue(Commands.runOnce(() -> shooter.setSpeed(50, 100, 0)));
    driverController.rightBumper().onTrue(Commands.runOnce(() -> shooter.setVolts(6, 2)));
    // driverController
    //     .rightBumper()
    //     .onTrue(
    //         IntakeCommands.Intake(
    //             intake, feederLower, feederUpper, wrist, () -> noteSensor.isHaveNote()));

    driverController
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(new Rotation2d(0)))))
                .ignoringDisable(true));
    driverController.start().onTrue(stopSubsystems().ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command stopSubsystems() {
    Command command =
        Commands.parallel(
            Commands.runOnce(() -> shooter.stop()),
            Commands.runOnce(() -> wrist.stop()),
            Commands.runOnce(() -> intake.stop()),
            Commands.runOnce(() -> feederLower.stop()),
            Commands.runOnce(() -> feederUpper.stop()));
    command.addRequirements(shooter, wrist, intake, feederLower, feederUpper);
    return command;
  }

  public void updateAlerts() {
    driverControllerDisconnectedAlert.set(!driverController.isConnected());
    OperatorControllerDisconnectedAlert.set(!operatorController.isConnected());

    if (noteSensor.isHaveNote()) {
      Logger.recordOutput(
          "RobotPose/Note",
          new Pose3d[] {
            new Pose3d(
                    drive.getPose().getX() + 0.0192236344,
                    drive.getPose().getY(),
                    0.3160213644,
                    new Rotation3d())
                .rotateAround(
                    new Translation3d(drive.getPose().getTranslation())
                        .plus(new Translation3d(-0.0153715466, 0.0, 0.2346029852)),
                    new Rotation3d(0, Units.degreesToRadians(-wrist.getPosition()), 0))
                .rotateAround(
                    new Translation3d(drive.getPose().getTranslation()),
                    new Rotation3d(drive.getPose().getRotation()))
          });
    } else {
      Logger.recordOutput("RobotPose/Note", new Pose3d[] {});
    }
  }
}
