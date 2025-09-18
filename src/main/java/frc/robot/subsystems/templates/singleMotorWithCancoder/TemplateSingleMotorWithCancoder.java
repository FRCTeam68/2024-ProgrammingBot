package frc.robot.subsystems.templates.singleMotorWithCancoder;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// TODO: update all of this
public class TemplateSingleMotorWithCancoder extends SubsystemBase {
  private final String name;
  private final TemplateSingleMotorWithCancoderIO io;
  protected final TemplateSinglemotorWithCancoderIOInputsAutoLogged inputs =
      new TemplateSinglemotorWithCancoderIOInputsAutoLogged();
  private final Alert disconnectedAlert;
  private final Alert tempAlert;

  // TODO: make this into a generic util
  private LoggedTunableNumber kP0 = new LoggedTunableNumber("ShooterUpper/Slot0Configs/kP", 5);
  private LoggedTunableNumber kD0 = new LoggedTunableNumber("ShooterUpper/Slot0Configs/kD", 0);
  private LoggedTunableNumber kS0 = new LoggedTunableNumber("ShooterUpper/Slot0Configs/kS", 0.5);
  private LoggedTunableNumber kV0 = new LoggedTunableNumber("ShooterUpper/Slot0Configs/kV", 0.2);
  private LoggedTunableNumber kA0 = new LoggedTunableNumber("ShooterUpper/Slot0Configs/kA", 0);
  private LoggedTunableNumber kG0 = new LoggedTunableNumber("ShooterUpper/Slot0Configs/kG", 0.5);

  private LoggedTunableNumber mmV =
      new LoggedTunableNumber("ShooterUpper/MotionMagicConfigs/Velocity", 40);
  private LoggedTunableNumber mmA =
      new LoggedTunableNumber("ShooterUpper/MotionMagicConfigs/Acceleration", 120);
  private LoggedTunableNumber mmJ =
      new LoggedTunableNumber("ShooterUpper/MotionMagicConfigs/Jerk", 400);

  private LoggedTunableNumber setpointBandSpeed =
      new LoggedTunableNumber("ShooterUpper/SetpointBandSpeed", 0.2);

  @Getter private double setpoint = 0.0;
  private double prevSetpoint = 0.0;

  @Getter private ControlMode controlMode = ControlMode.Neutral;
  private ControlMode prevControlMode = ControlMode.Neutral;

  private LoggedNetworkNumber testSetpoint =
      new LoggedNetworkNumber("SmartDashboard/ShooterUpper/RunToSpeed/TestSetpoint", 0);

  public TemplateSingleMotorWithCancoder(String name, TemplateSingleMotorWithCancoderIO io) {
    this.name = name;
    this.io = io;

    disconnectedAlert = new Alert(name + " shooter motor disconnected.", Alert.AlertType.kError);
    tempAlert = new Alert(name + " shooter motor high temp.", Alert.AlertType.kWarning);

    setPID();
    setMotionMagic();

    // Dashboard tuning commands
    SmartDashboard.putData(
        "Shooter" + name + "/RunToSpeed/RunToTestSetpoint",
        Commands.runOnce(() -> setSpeed(testSetpoint.get())));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter" + name, inputs);
    disconnectedAlert.set(!inputs.talonConnected);
    tempAlert.set(inputs.tempCelsius > Constants.warningTemp);
    Logger.recordOutput("Shooter" + name + "/AtSetpoint", atSetpoint());

    if (controlMode != prevControlMode || setpoint != prevSetpoint) {
      prevControlMode = controlMode;
      prevSetpoint = setpoint;

      Logger.recordOutput(
          "Shooter" + name + "/setpointVolts", (controlMode == ControlMode.Voltage) ? setpoint : 0);
      Logger.recordOutput(
          "Shooter" + name + "/setpointSpeed", (controlMode == ControlMode.Speed) ? setpoint : 0);
    }

    // Update tunable numbers
    if (Constants.tuningMode) {
      if (kP0.hasChanged(hashCode())
          || kD0.hasChanged(hashCode())
          || kS0.hasChanged(hashCode())
          || kV0.hasChanged(hashCode())
          || kA0.hasChanged(hashCode())
          || kG0.hasChanged(hashCode())) {
        setPID();
      }

      if (mmV.hasChanged(hashCode()) || mmA.hasChanged(hashCode()) || mmJ.hasChanged(hashCode())) {
        setMotionMagic();
      }
    }
  }

  public void setAtSetpointBandSpeed(LoggedTunableNumber band) {
    setpointBandSpeed = band;
  }

  /**
   * Set applied voltage to the motor
   *
   * @param inputVolts Voltage to drive motor at
   */
  public void setVolts(double volts) {
    setpoint = volts;
    controlMode = ControlMode.Voltage;
    io.setVolts(volts);
  }

  /**
   * Set goal speed of the mechanism in degrees of elevation per second
   *
   * @param speed Goal speed
   */
  public void setSpeed(double speed) {
    setpoint = speed;
    controlMode = ControlMode.Speed;
    io.setSpeed(speed, 0);
  }

  /**
   * Velocity of the mechanism in degrees of elevation per second
   *
   * @return Velocity
   */
  public double getSpeed() {
    return inputs.velocityRotsPerSec;
  }

  /**
   * Current corresponding to the torque output by the lead motor. Similar to StatorCurrent. Users
   * will likely prefer this current to calculate the applied torque to the rotor.
   *
   * <p>Stator current where positive current means torque is applied in the forward direction as
   * determined by the Inverted setting.
   *
   * @return motor torque current
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  /**
   * Check if mechanism is at goal speed with error of setpointBandSpeed
   *
   * @return True if in speed control mode and mechanism is at goal speed, false otherwise
   */
  public boolean atSetpoint() {
    switch (controlMode) {
      case Speed:
        return Math.abs(setpoint - inputs.velocityRotsPerSec) < setpointBandSpeed.getAsDouble();
      default:
        return false;
    }
  }

  /** Stop motor */
  public void stop() {
    controlMode = ControlMode.Neutral;
    io.stop();
  }

  private void setPID() {
    io.setPID(
        new SlotConfigs()
            .withKP(kP0.getAsDouble())
            .withKD(kD0.getAsDouble())
            .withKS(kS0.getAsDouble())
            .withKV(kV0.getAsDouble())
            .withKA(kA0.getAsDouble())
            .withKG(kG0.getAsDouble()));
  }

  private void setMotionMagic() {
    io.setMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(mmV.getAsDouble())
            .withMotionMagicAcceleration(mmA.getAsDouble())
            .withMotionMagicJerk(mmJ.getAsDouble()));
  }
}
