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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.auton.AutonCommands;
import frc.robot.commands.auton.AutonSequence;
import frc.robot.commands.auton.AutonSequenceCenter;
import frc.robot.commands.auton.AutonSequenceSide;
import frc.robot.subsystems.NoteVisualizer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOReal;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import frc.robot.subsystems.sensors.NoteSensor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraInfo;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutonUtil;
import frc.robot.util.FollowPathUtil;
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
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandPS4Controller operatorController = new CommandPS4Controller(1);

  // Alerts
  private final Alert driverControllerDisconnectedAlert =
      new Alert("Driver Xbox controller disconnected.", AlertType.kError);
  private final Alert operatorControllerDisconnectedAlert =
      new Alert("Operator PS4 controller disconnected.", AlertType.kError);
  private final Alert noAutoSelectedAlert =
      new Alert("No autonomous selected.", AlertType.kWarning);
  private final Alert startingPoseAlert =
      new Alert(
          "Current robot pose is too far from starting pose for selected auton. Possible causes include the incorrect auton is selected, the camera is not getting a clear view of an april tag, or the robot is in the wrong location.",
          AlertType.kError);

  // Dashboard inputs
  private final LoggedDashboardChooser<AutonSequence> autonChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOReal(DriveConstants.moduleConfigs[0]),
                new ModuleIOReal(DriveConstants.moduleConfigs[1]),
                new ModuleIOReal(DriveConstants.moduleConfigs[2]),
                new ModuleIOReal(DriveConstants.moduleConfigs[3]));

        vision = new Vision(drive::addVisionMeasurement, drive::getPose, drive::getFieldVelocity);
        // new VisionIOLimelight(
        //     CameraType.LL_2, VisionConstants.LL2Name, drive::getRotation));

        // wrist = new Wrist(drive::getPose, new WristIOReal());
        wrist = new Wrist(drive::getPose, new WristIO() {});

        shooter =
            new Shooter(
                new ShooterIOReal(30, InvertedValue.Clockwise_Positive),
                new ShooterIOReal(31, InvertedValue.CounterClockwise_Positive));

        intake =
            new RollerSystem(
                "Intake",
                new RollerSystemIOTalonFX(
                    20,
                    "rio",
                    40,
                    InvertedValue.CounterClockwise_Positive,
                    NeutralModeValue.Coast,
                    1));

        feederLower =
            new RollerSystem(
                "FeederLower",
                new RollerSystemIOTalonFX(
                    35,
                    "rio",
                    40,
                    InvertedValue.CounterClockwise_Positive,
                    NeutralModeValue.Coast,
                    1));
        feederUpper =
            new RollerSystem(
                "FeederUpper",
                new RollerSystemIOTalonFX(
                    36, "rio", 40, InvertedValue.Clockwise_Positive, NeutralModeValue.Brake, 1));
      }
      case SIM -> {
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
                new VisionIOLimelight(CameraInfo.LL_4));

        wrist = new Wrist(drive::getPose, new WristIOSim());

        shooter = new Shooter(new ShooterIOSim(), new ShooterIOSim());

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
                new VisionIOLimelight(CameraInfo.LL_4));

        wrist = new Wrist(drive::getPose, new WristIO() {});

        shooter = new Shooter(new ShooterIO() {}, new ShooterIO() {});

        intake = new RollerSystem("Intake", new RollerSystemIO() {});

        feederLower = new RollerSystem("FeederLower", new RollerSystemIO() {});

        feederUpper = new RollerSystem("FeederUpper", new RollerSystemIO() {});
      }
    }

    intake.initPID(new SlotConfigs().withKP(5).withKD(0).withKS(0));

    feederLower.initPID(new SlotConfigs().withKP(10).withKD(0).withKS(0));

    feederUpper.initPID(new SlotConfigs().withKP(15).withKD(0).withKS(0));

    noteSensor = new NoteSensor();

    noteVisualizer = new NoteVisualizer(drive::getPose, wrist::getPosition, noteSensor::isDetected);

    // Set up dashboard auto chooser
    autonChooser = new LoggedDashboardChooser<>("Auto Chooser");
    autonChooser.addDefaultOption("NONE", null);
    autonChooser.addOption("Center", new AutonSequenceCenter());
    autonChooser.addOption("Left", new AutonSequenceSide(0));
    autonChooser.addOption("Right", new AutonSequenceSide(1));

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
        .start()
        .onTrue(
            Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            drive.getPose().getTranslation(),
                            AllianceFlipUtil.shouldFlip()
                                ? new Rotation2d()
                                : new Rotation2d(Math.PI)))));

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
        .onTrue(Commands.runOnce(() -> shooter.runVelocity(50, 0, 100, 0)));
    driverController.rightBumper().onTrue(Commands.runOnce(() -> shooter.runVolts(6, 2)));
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
    driverController.start().onTrue(Commands.runOnce(() -> stopSubsystems()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutonCommands.autonCommand(
        drive, wrist, shooter, intake, feederLower, feederUpper, noteSensor, autonChooser.get());
  }

  /** Loads autonomous paths from storage. This method can be safely be called periodically. */
  public void loadAutonomousPath() {
    AutonUtil.loadPaths(autonChooser.get() != null ? autonChooser.get().getPathNames() : null);
  }

  /** Stops all subsystems and cancels any scheduled commands. */
  public void stopSubsystems() {
    CommandScheduler.getInstance().cancelAll();
    drive.stop();
    shooter.stop();
    wrist.stop();
    intake.stop();
    feederLower.stop();
    feederUpper.stop();
  }

  /**
   * Alerts always active:
   *
   * <ul>
   *   <li>Controllers disconnected
   * </ul>
   *
   * Alerts only active while in autonomous and disabled:
   *
   * <ul>
   *   <li>No autonomous selected
   *   <li>Current pose does not match autonomous starting pose
   * </ul>
   */
  public void updateAlerts() {
    driverControllerDisconnectedAlert.set(!driverController.isConnected());
    operatorControllerDisconnectedAlert.set(!operatorController.isConnected());

    if (DriverStation.isAutonomous() && DriverStation.isDisabled()) {
      noAutoSelectedAlert.set(autonChooser.get() == null);
      startingPoseAlert.set(
          autonChooser.get() != null
              && (FollowPathUtil.getStartingPose().minus(drive.getPose()).getTranslation().getNorm()
                      > 0.25
                  || FollowPathUtil.getStartingPose()
                          .minus(drive.getPose())
                          .getRotation()
                          .getDegrees()
                      > 20));
    } else {
      noAutoSelectedAlert.set(false);
      startingPoseAlert.set(false);
    }
  }
}
