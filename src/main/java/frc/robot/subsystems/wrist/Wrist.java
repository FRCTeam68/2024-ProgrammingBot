package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  @Getter private static final double minimum = 10;
  @Getter private static final double maximum = 45;
  @Getter private static final double startingElevation = 45;

  @Getter
  private static final LoggedTunableNumber intake =
      new LoggedTunableNumber("Wrist/IntakeElevation", 40);

  @Getter
  private static final LoggedTunableNumber outtake =
      new LoggedTunableNumber("Wrist/OuttakeElevation", 35);

  // Tunable numbers
  private LoggedTunableNumber kP = new LoggedTunableNumber("Wrist/kP", 1100);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Wrist/kD", 0);
  private LoggedTunableNumber kS = new LoggedTunableNumber("Wrist/kS", 0.227);
  private LoggedTunableNumber kG = new LoggedTunableNumber("Wrist/kG", 0.03);

  private LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("Wrist/PositionSetpointBandDegrees", 3);

  private final WristIO io;
  protected final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  // Alerts
  private final Alert leaderDisconnectedAlert =
      new Alert("Left wrist motor disconnected.", AlertType.kError);
  private final Alert followerDisconnectedAlert =
      new Alert("Right wrist motor disconnected.", AlertType.kError);
  private final Alert leaderTempAlert =
      new Alert("Left wrist motor is too hot!", AlertType.kWarning);
  private final Alert followerTempAlert =
      new Alert("Right wrist motor is too hot!", AlertType.kWarning);

  // Connected debouncers
  private final Debouncer connectedDebouncer = new Debouncer(0.5);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5);

  private double setpoint = 0.0;

  private ControlMode mode = ControlMode.Neutral;

  private Pose3d notePose = new Pose3d();
  private final Supplier<Pose2d> drivePoseSupplier;

  public Wrist(Supplier<Pose2d> drivePoseSupplier, WristIO io) {
    this.drivePoseSupplier = drivePoseSupplier;
    this.io = io;

    setPosition(startingElevation);

    SmartDashboard.putNumber("Wrist/Manual Position", startingElevation);
    SmartDashboard.putData(
        "Wrist/Run To Manual Position",
        Commands.runOnce(
            () -> runPosition(SmartDashboard.getNumber("Wrist/Manual Position", startingElevation)),
            this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    leaderDisconnectedAlert.set(!connectedDebouncer.calculate(inputs.leaderConnected));
    followerDisconnectedAlert.set(!followerConnectedDebouncer.calculate(inputs.followerConnected));
    leaderTempAlert.set(inputs.leaderTempCelsius > Constants.warningTempCelsius);
    followerTempAlert.set(inputs.followerTempCelsius > Constants.warningTempCelsius);

    Logger.recordOutput(
        "RobotPose/Wrist",
        new Pose3d(
            new Translation3d(-0.0153715466, 0, 0.2346029852),
            new Rotation3d(0, Units.degreesToRadians(-inputs.elevationDeg), 0)));

    // TODO: should this be in robotState
    notePose =
        new Pose3d(
                drivePoseSupplier.get().getX() + 0.0192236344,
                drivePoseSupplier.get().getY(),
                0.3160213644,
                new Rotation3d())
            .rotateAround(
                new Translation3d(drivePoseSupplier.get().getTranslation())
                    .plus(new Translation3d(-0.0153715466, 0.0, 0.2346029852)),
                new Rotation3d(0, Units.degreesToRadians(-inputs.elevationDeg), 0))
            .rotateAround(
                new Translation3d(drivePoseSupplier.get().getTranslation()),
                new Rotation3d(drivePoseSupplier.get().getRotation()));
    Logger.recordOutput("RobotPose/HaveNote", RobotState.haveNote ? notePose : null);

    // Update tunable numbers
    if (kP.hasChanged(hashCode())
        || kD.hasChanged(hashCode())
        || kS.hasChanged(hashCode())
        || kG.hasChanged(hashCode())) {
      io.setPID(
          new SlotConfigs().withKP(kP.get()).withKD(kD.get()).withKS(kS.get()).withKG(kG.get()));
    }
  }

  /**
   * Set applied voltage to the motor
   *
   * @param inputVolts Voltage to drive motor at
   */
  public void runVolts(double volts) {
    mode = ControlMode.Voltage;
    io.runVolts(volts);
  }

  /**
   * Set goal position of the mechanism in degrees of elevation
   *
   * @param elevation Goal position
   */
  public void runPosition(double elevation) {
    setpoint = elevation;
    mode = ControlMode.Position;
    io.runPosition(setpoint);
    Logger.recordOutput("Wrist/SetpointElevationDeg", setpoint);
  }

  /** Stop motor */
  public void stop() {
    mode = ControlMode.Neutral;
    io.stop();
  }

  /**
   * Set the current mechanism in degrees of elevation
   *
   * @param elevation current elevation
   */
  public void setPosition(double elevation) {
    io.setPosition(elevation);
  }

  /** Enables the software motion limits */
  public void setSoftwareLimitsEnabled(boolean enabled) {
    io.setSoftwareLimitsEnabled(enabled);
  }

  /**
   * Velocity of the mechanism in degrees of elevation per second
   *
   * @return Velocity
   */
  public double getVelocity() {
    return inputs.velocityDegPerSec;
  }

  /**
   * Position of the mechanism in degrees of elevation
   *
   * @return Elevation of the wrist
   */
  public double getPosition() {
    return inputs.elevationDeg;
  }

  /**
   * Current corresponding to the torque output by the lead motor. Similar to StatorCurrent. Users
   * will likely prefer this current to calculate the applied torque to the rotor.
   *
   * <p>Stator current where positive current means torque is applied in the forward direction as
   * determined by the Inverted setting.
   *
   * @return Lead motor torque current
   */
  public double getLeaderTorqueCurrent() {
    return inputs.leaderTorqueCurrentAmps;
  }

  /**
   * Current corresponding to the torque output by the follower motor. Similar to StatorCurrent.
   * Users will likely prefer this current to calculate the applied torque to the rotor.
   *
   * <p>Stator current where positive current means torque is applied in the forward direction as
   * determined by the Inverted setting.
   *
   * @return Follower motor torque current
   */
  public double getFollowerTorqueCurrent() {
    return inputs.followerTorqueCurrentAmps;
  }

  /**
   * Check if mechanism is at goal position with error of setpointBandPosition
   *
   * @return True if in position control mode and mechanism is at goal position, false otherwise
   */
  @AutoLogOutput(key = "Wrist/atSetpoint")
  public boolean atSetpoint() {
    return (mode == ControlMode.Position)
        ? Math.abs(setpoint - inputs.elevationDeg) < setpointBandPosition.getAsDouble()
        : false;
  }
}
