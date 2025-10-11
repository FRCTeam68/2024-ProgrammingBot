package frc.robot;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.NoteVisualizer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOComp;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import frc.robot.subsystems.sensors.NoteSensor;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIO.CameraType;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Wrist wrist;
  private Shooter shooter;
  private RollerSystem intake;
  private RollerSystem feederLower;
  private RollerSystem feederUpper;
  private NoteSensor noteSensor;
  private NoteVisualizer noteVisualizer;

  // Controllers
  private static final CommandXboxController driverController = new CommandXboxController(0);
  private static final CommandPS4Controller operatorController = new CommandPS4Controller(1);

  // Alerts
  private static final Alert driverControllerDisconnectedAlert =
      new Alert("Driver Xbox controller disconnected.", AlertType.kError);
  private static final Alert operatorControllerDisconnectedAlert =
      new Alert("Operator PS4 controller disconnected.", AlertType.kError);
  private static final Alert noAutoSelectedAlert =
      new Alert("Please select an auton.", AlertType.kWarning);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOComp(DriveConstants.moduleConfigs[0]),
                new ModuleIOComp(DriveConstants.moduleConfigs[1]),
                new ModuleIOComp(DriveConstants.moduleConfigs[2]),
                new ModuleIOComp(DriveConstants.moduleConfigs[3]));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                drive::getFieldVelocity,
                new VisionIOLimelight(
                    CameraType.LL_2, VisionConstants.LL2Name, drive::getRotation));

        wrist = new Wrist(new WristIOTalonFX());

        shooter =
            new Shooter(
                new RollerSystemIOTalonFX(
                    20, "rio", 80, InvertedValue.Clockwise_Positive, NeutralModeValue.Coast, 1),
                new RollerSystemIOTalonFX(
                    21,
                    "rio",
                    80,
                    InvertedValue.CounterClockwise_Positive,
                    NeutralModeValue.Coast,
                    1));

        intake =
            new RollerSystem(
                "Intake",
                new RollerSystemIOTalonFX(
                    17, "rio", 40, InvertedValue.Clockwise_Positive, NeutralModeValue.Coast, 1));

        feederLower =
            new RollerSystem(
                "FeederLower",
                new RollerSystemIOTalonFX(
                    18, "rio", 40, InvertedValue.Clockwise_Positive, NeutralModeValue.Brake, 1));
        feederUpper =
            new RollerSystem(
                "FeederUpper",
                new RollerSystemIOTalonFX(
                    19, "rio", 40, InvertedValue.Clockwise_Positive, NeutralModeValue.Brake, 1));
      }
      case SIM -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision = new Vision(drive::addVisionMeasurement, drive::getPose, drive::getFieldVelocity);

        wrist = new Wrist(new WristIOSim());

        shooter =
            new Shooter(
                new RollerSystemIOSim(DCMotor.getFalcon500Foc(1), 1, 0.1),
                new RollerSystemIOSim(DCMotor.getFalcon500Foc(1), 1, 0.1));

        intake =
            new RollerSystem("Intake", new RollerSystemIOSim(DCMotor.getFalcon500Foc(1), 1, .3));

        feederLower =
            new RollerSystem(
                "FeederLower", new RollerSystemIOSim(DCMotor.getFalcon500Foc(1), 1, .074));
        feederUpper =
            new RollerSystem(
                "FeederUpper", new RollerSystemIOSim(DCMotor.getFalcon500Foc(1), 1, .074));
      }
      case REPLAY -> {
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

        shooter = new Shooter(new RollerSystemIO() {}, new RollerSystemIO() {});

        intake = new RollerSystem("Intake", new RollerSystemIO() {});

        feederLower = new RollerSystem("FeederLower", new RollerSystemIO() {});

        feederUpper = new RollerSystem("FeederUpper", new RollerSystemIO() {});
      }
    }

    intake.setPID(new SlotConfigs().withKP(5).withKD(0).withKS(0));

    feederLower.setPID(new SlotConfigs().withKP(10).withKD(0).withKS(0));

    feederUpper.setPID(new SlotConfigs().withKP(15).withKD(0).withKS(0));

    noteSensor = new NoteSensor();

    noteVisualizer = new NoteVisualizer(drive::getPose, wrist::getPosition, noteSensor::isHaveNote);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

    // Set up auto routines
    autoChooser.addOption("None", Commands.none());

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Use this method to define button -> command mappings. */
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
            DriveCommands.joystickDriveAtTarget(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Translation2d(1, 1)));
    // noteSensor.automaticSimulatedNote(3))
    // .onlyIf(
    //     () ->
    //         Constants.getMode() == Mode.SIM && noteSensor.getAutomaticNoteSim().get()));
    driverController
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> shooter.setVelocity(50, 0, 100, 0)));
    driverController.rightBumper().onTrue(Commands.runOnce(() -> shooter.setVolts(6, 2)));
    // driverController
    //     .rightBumper()
    //     .onTrue(
    //         IntakeCommands.Intake(
    //             intake, feederLower, feederUpper, wrist, () -> noteSensor.isHaveNote()));

    driverController
        .povLeft()
        .whileTrue(
            DriveCommands.AutopilotDriveToPose(
                drive, () -> new Pose2d(8, 1, new Rotation2d()), null));

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
    return Commands.runOnce(
        () -> {
          shooter.stop();
          wrist.stop();
          intake.stop();
          feederLower.stop();
          feederUpper.stop();
        },
        shooter,
        wrist,
        intake,
        feederLower,
        feederUpper);
  }

  public static void controllerAlerts() {
    driverControllerDisconnectedAlert.set(!driverController.isConnected());
    operatorControllerDisconnectedAlert.set(!operatorController.isConnected());
  }
}
