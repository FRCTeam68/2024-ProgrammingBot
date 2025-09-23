package frc.robot.subsystems.Devbot.wrist;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
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
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  protected final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private final Alert leaderDisconnectedAlert =
      new Alert("Lead wrist motor disconnected.", AlertType.kError);
  private final Alert followerDisconnectedAlert =
      new Alert("Follower wrist motor disconnected.", AlertType.kError);
  private final Alert motorsOutOfSync = new Alert("Wrist motors out of sync", AlertType.kWarning);
  private final Alert leaderTempAlert =
      new Alert("Lead wrist motor (left) is too hot!", AlertType.kWarning);
  private final Alert followerTempAlert =
      new Alert("Follower wrist motor (right) is too hot!", AlertType.kWarning);

  private LoggedTunableNumber kP0 = new LoggedTunableNumber("Wrist/Slot0/kP", 10);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("Wrist/Slot0/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("Wrist/Slot0/kS", 0.5);
  private LoggedTunableNumber kG0 = new LoggedTunableNumber("Wrist/Slot0/kG", 0);

  private LoggedTunableNumber mmV = new LoggedTunableNumber("Wrist/MotionMagic/Velocity", 100);
  private LoggedTunableNumber mmA = new LoggedTunableNumber("Wrist/MotionMagic/Acceleration", 120);

  private LoggedTunableNumber setpointBandPosition =
      new LoggedTunableNumber("Wrist/PositionSetpointBandPosition", 0.2);

  @Getter private double setpoint = 0.0;

  @Getter private ControlMode mode = ControlMode.Neutral;

  private LoggedNetworkNumber testSetpoint =
      new LoggedNetworkNumber("Testing/Wrist/RunToPosition/TestSetpoint", 0);

  public Wrist(WristIO io) {
    this.io = io;

    setPID();
    setMotionMagic();
    zero();

    // Dashboard tuning commands

    SmartDashboard.putData("Wrist/Zero", Commands.runOnce(() -> zero()));
    SmartDashboard.putData(
        "Wrist/RunToPosition/RunToTestSetpoint",
        Commands.runOnce(() -> setPosition(testSetpoint.get())));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    leaderDisconnectedAlert.set(!inputs.leaderConnected);
    followerDisconnectedAlert.set(!inputs.followerConnected);
    motorsOutOfSync.set(inputs.elevationOffsetDeg > 2);
    leaderTempAlert.set(inputs.leaderTempCelsius > Constants.warningTemp);
    followerTempAlert.set(inputs.followerTempCelsius > Constants.warningTemp);
    Logger.recordOutput("Wrist/AtPosition", atPosition());

    Logger.recordOutput("Wrist/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput("Wrist/SetpointRotsPerSec", (mode == ControlMode.Speed) ? setpoint : 0);
    Logger.recordOutput(
        "Wrist/SetpointElevationDeg", (mode == ControlMode.Position) ? setpoint : 0);

    Logger.recordOutput(
        "RobotPose/Wrist",
        new Pose3d(
            new Translation3d(-0.0153715466, 0, 0.2346029852),
            new Rotation3d(0, Units.degreesToRadians(-inputs.elevationDeg), 0)));

    // Update tunable numbers
    if (Constants.tuningMode) {
      if (kP0.hasChanged(hashCode())
          || kD0.hasChanged(hashCode())
          || kS0.hasChanged(hashCode())
          || kG0.hasChanged(hashCode())) {
        setPID();
      }

      if (mmV.hasChanged(hashCode()) || mmA.hasChanged(hashCode())) {
        setMotionMagic();
      }
    }
  }

  public void setAtSetpointBandPosition(LoggedTunableNumber band) {
    setpointBandPosition = band;
  }

  /**
   * Set applied voltage to the motor
   *
   * @param inputVolts Voltage to drive motor at
   */
  public void setVolts(double volts) {
    setpoint = volts;
    mode = ControlMode.Voltage;
    io.setVolts(volts);
  }

  /**
   * Set goal speed of the mechanism in degrees of elevation per second
   *
   * @param speed Goal speed
   */
  public void setSpeed(double speed) {
    setpoint = speed;
    mode = ControlMode.Speed;
    io.setSpeed(speed, 0);
  }

  /**
   * Set goal position of the mechanism in degrees of elevation
   *
   * @param elevation Goal position
   */
  public void setPosition(double elevation) {
    setpoint = elevation;
    mode = ControlMode.Position;
    io.setPosition(elevation, 0);
  }

  /**
   * Velocity of the mechanism in degrees of elevation per second
   *
   * @return Velocity
   */
  public double getSpeed() {
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
  public boolean atPosition() {
    return (mode == ControlMode.Position)
        ? Math.abs(setpoint - inputs.elevationDeg) < setpointBandPosition.getAsDouble()
        : false;
  }

  /** Stop motor */
  public void stop() {
    mode = ControlMode.Neutral;
    io.stop();
  }

  /** Sets the current mechanism position to zero */
  public void zero() {
    io.zero(55);
  }

  private void setPID() {
    io.setPID(
        new SlotConfigs()
            .withKP(kP0.getAsDouble())
            .withKD(kD0.getAsDouble())
            .withKS(kS0.getAsDouble())
            .withKG(kG0.getAsDouble()));
  }

  private void setMotionMagic() {
    io.setMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(mmV.getAsDouble())
            .withMotionMagicAcceleration(mmA.getAsDouble()));
  }
}
