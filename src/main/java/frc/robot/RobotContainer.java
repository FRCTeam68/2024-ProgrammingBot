package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.auton.AutonCommands;
import frc.robot.commands.auton.AutonSequence;
import frc.robot.commands.auton.AutonSequenceCenter;
import frc.robot.commands.auton.AutonSequenceSide;
import frc.robot.subsystems.ShooterConstants.ShooterConfig;
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
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOReal;
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
  // private NoteVisualizer noteVisualizer;

  // Controllers
  // TODO: do we drop the controller part
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandPS4Controller operatorController = new CommandPS4Controller(1);

  // Alerts
  private final Alert driverControllerDisconnectedAlert =
      new Alert("Driver Xbox controller disconnected.", AlertType.kError);
  private final Alert operatorControllerDisconnectedAlert =
      new Alert("Operator PS4 controller disconnected.", AlertType.kError);
  private final Alert noAutoSelectedAlert =
      new Alert("No autonomous routine selected.", AlertType.kWarning);
  private final Alert startingPoseAlert =
      new Alert(
          "Current robot pose does not match the starting pose for selected auton. Possible causes include the incorrect auton is selected, the camera is not getting a clear view of an april tag, or the robot is in the wrong location.",
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                drive::getFieldVelocity,
                new VisionIOLimelight(CameraInfo.LL_4));

        wrist = new Wrist(drive::getPose, new WristIOReal());

        shooter =
            new Shooter(
                new ShooterIOReal(30, InvertedValue.CounterClockwise_Positive),
                new ShooterIOReal(31, InvertedValue.CounterClockwise_Positive));

        intake =
            new RollerSystem(
                "Intake",
                new RollerSystemIOTalonFX(
                    20,
                    new CANBus("rio"),
                    40,
                    InvertedValue.CounterClockwise_Positive,
                    NeutralModeValue.Coast,
                    1));

        feederLower =
            new RollerSystem(
                "FeederLower",
                new RollerSystemIOTalonFX(
                    35,
                    new CANBus("rio"),
                    40,
                    InvertedValue.CounterClockwise_Positive,
                    NeutralModeValue.Coast,
                    1));

        feederUpper =
            new RollerSystem(
                "FeederUpper",
                new RollerSystemIOTalonFX(
                    36,
                    new CANBus("rio"),
                    40,
                    InvertedValue.Clockwise_Positive,
                    NeutralModeValue.Brake,
                    1));
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
                new VisionIO() {});

        wrist = new Wrist(drive::getPose, new WristIO() {});

        shooter = new Shooter(new ShooterIO() {}, new ShooterIO() {});

        intake = new RollerSystem("Intake", new RollerSystemIO() {});

        feederLower = new RollerSystem("FeederLower", new RollerSystemIO() {});

        feederUpper = new RollerSystem("FeederUpper", new RollerSystemIO() {});
      }
    }

    noteSensor = new NoteSensor();

    intake.initPID(new SlotConfigs().withKP(5).withKD(0).withKS(0));

    feederLower.initPID(new SlotConfigs().withKP(10).withKD(0).withKS(0));

    feederUpper.initPID(new SlotConfigs().withKP(15).withKD(0).withKS(0));

    // noteVisualizer = new NoteVisualizer(drive::getPose, wrist::getPosition,
    // noteSensor::isDetected);

    // Configure the button bindings
    configureButtonBindings();

    // Configure dashboard
    SmartDashboard.putData(
        "Move To Starting Pose",
        Commands.runOnce(() -> wrist.runPosition(Wrist.getStartingElevation()), wrist)
            .andThen(() -> stopSubsystems()));
    SmartDashboard.putData("Wrist/Find Limit", ManipulatorCommands.wristZeroByAmps(wrist));

    // Configure auton chooser
    autonChooser = new LoggedDashboardChooser<>("Auton Chooser");
    autonChooser.addDefaultOption("NONE", null);
    autonChooser.addOption("Center", new AutonSequenceCenter());
    autonChooser.addOption("Left", new AutonSequenceSide(0));
    autonChooser.addOption("Right", new AutonSequenceSide(1));

    SmartDashboard.putData(
        "Test/autopilot",
        DriveCommands.autopilotDriveToPose(
            drive,
            () ->
                new APTarget(new Pose2d(2, 2, new Rotation2d(1)))
                    .withEntryAngle(new Rotation2d(-1))
                    .withVelocity(0.5)));

    SmartDashboard.putData(
        "Test/autopilot2",
        DriveCommands.autopilotDriveToPose(
            drive,
            () ->
                new APTarget(new Pose2d(0, 0, new Rotation2d(2)))
                    .withEntryAngle(new Rotation2d())));
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

    // driverController.povUp().onTrue(TestCommands.subsystem1(intake, feederLower, feederUpper));
    // driverController.povLeft().onTrue(TestCommands.subsystem2(intake, feederLower, feederUpper));
    // driverController.povRight().onTrue(TestCommands.subsystem3(intake, feederLower,
    // feederUpper));
    // driverController.povDown().onTrue(TestCommands.subsystem4(intake, feederLower, feederUpper));

    driverController
        .b()
        .onTrue(
            Commands.deadline(
                IntakeCommands.intake(wrist, intake, feederLower, feederUpper, noteSensor),
                // TODO: does this work how I think
                DriveCommands.joystickDriveAtTarget(
                        drive,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> vision.getTargetNote().get().getTranslation())
                    .onlyWhile(() -> vision.getTargetNote().isPresent())));

    driverController
        .a()
        .onTrue(
            ShootCommands.setStaticShotConfig(
                shooter, wrist, new ShooterConfig(Wrist.getStartingElevation(), 0, 70)));

    driverController
        .x()
        .onTrue(
            ShootCommands.setStaticShotConfig(
                shooter, wrist, new ShooterConfig(Wrist.getStartingElevation(), 0, 10)));

    driverController
        .leftTrigger()
        .onTrue(IntakeCommands.intake(wrist, intake, feederLower, feederUpper, noteSensor));

    driverController
        .leftBumper()
        .whileTrue(IntakeCommands.outtake(wrist, intake, feederLower, feederUpper));

    driverController.rightTrigger().whileTrue(ShootCommands.shootManual(feederUpper));

    driverController
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(),
                                AllianceFlipUtil.apply(new Rotation2d()))))
                .ignoringDisable(true));

    driverController.start().onTrue(Commands.runOnce(() -> stopSubsystems()).ignoringDisable(true));
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

  public void setCameraThrottle(int skippedFrames) {
    vision.setThrottle(skippedFrames);
  }

  /** Stops all subsystems and cancels all scheduled commands. */
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
