package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO upperIO;
  private final ShooterIO lowerIO;
  protected final ShooterIOInputsAutoLogged upperInputs = new ShooterIOInputsAutoLogged();
  protected final ShooterIOInputsAutoLogged lowerInputs = new ShooterIOInputsAutoLogged();

  // Alerts
  private final Alert upperDisconnectedAlert =
      new Alert("Upper shooter motor (Right) disconnected.", AlertType.kError);
  private final Alert lowerDisconnectedAlert =
      new Alert("Lower shooter motor (Left) disconnected.", AlertType.kError);
  private final Alert upperTempAlert =
      new Alert("Upper shooter motor (Right) is too hot!", AlertType.kWarning);
  private final Alert lowerTempAlert =
      new Alert("Lower shooter motor (Left) is too hot!", AlertType.kWarning);

  // TODO: make this into a generic util
  // PID
  private final String pidKey = "Shooter/Slot";
  private LoggedTunableNumber[] kP = {
    new LoggedTunableNumber(pidKey + "0/kP"), new LoggedTunableNumber(pidKey + "1/kP")
  };
  private LoggedTunableNumber[] kD = {
    new LoggedTunableNumber(pidKey + "0/kD"), new LoggedTunableNumber(pidKey + "1/kD")
  };
  private LoggedTunableNumber[] kS = {
    new LoggedTunableNumber(pidKey + "0/kS"), new LoggedTunableNumber(pidKey + "1/kS")
  };
  private LoggedTunableNumber mmA = new LoggedTunableNumber("Shooter/MotionMagic/Acceleration");
  private LoggedTunableNumber setpointBandSpeed =
      new LoggedTunableNumber("Shooter/AtSetpointError");

  {
    kP[0].initDefault(5);
    kD[0].initDefault(5);
    kS[0].initDefault(0);
    kP[1].initDefault(5);
    kD[1].initDefault(5);
    kS[1].initDefault(0);
    mmA.initDefault(100);
    setpointBandSpeed.initDefault(1);
  }

  @Getter private double upperSetpoint = 0.0;
  @Getter private double lowerSetpoint = 0.0;

  @Getter private ControlMode controlMode = ControlMode.Neutral;

  public Shooter(ShooterIO lowerIO, ShooterIO upperIO) {
    this.upperIO = upperIO;
    this.lowerIO = lowerIO;
  }

  public void periodic() {
    upperIO.updateInputs(upperInputs);
    lowerIO.updateInputs(lowerInputs);
    Logger.processInputs("Shooter/Upper", upperInputs);
    Logger.processInputs("Shooter/Lower", lowerInputs);
    upperDisconnectedAlert.set(!upperInputs.connected);
    lowerDisconnectedAlert.set(!lowerInputs.connected);
    upperTempAlert.set(upperInputs.tempCelsius > Constants.warningTempCelsius);
    lowerTempAlert.set(lowerInputs.tempCelsius > Constants.warningTempCelsius);

    Logger.recordOutput(
        "Shooter/Upper/SetpointVolts", (controlMode == ControlMode.Voltage) ? upperSetpoint : 0);
    Logger.recordOutput(
        "Shooter/Upper/SetpointRotsPerSec",
        (controlMode == ControlMode.Velocity) ? upperSetpoint : 0);
    Logger.recordOutput(
        "Shooter/Lower/SetpointVolts", (controlMode == ControlMode.Voltage) ? lowerSetpoint : 0);
    Logger.recordOutput(
        "Shooter/Lower/SetpointRotsPerSec",
        (controlMode == ControlMode.Velocity) ? lowerSetpoint : 0);

    // Update tunable numbers
    if (Constants.tuningMode) {
      if (kP[0].hasChanged(hashCode())
          || kD[0].hasChanged(hashCode())
          || kS[0].hasChanged(hashCode())
          || kP[1].hasChanged(hashCode())
          || kD[1].hasChanged(hashCode())
          || kS[1].hasChanged(hashCode())) {
        setPID();
      }

      if (mmA.hasChanged(hashCode())) {
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
  public void runVolts(double upperVolts, double lowerVolts) {
    upperSetpoint = upperVolts;
    lowerSetpoint = lowerVolts;
    controlMode = ControlMode.Voltage;
    upperIO.runVolts(upperVolts);
    lowerIO.runVolts(lowerVolts);
  }

  /**
   * Set goal velocity of the mechanism in degrees of elevation per second
   *
   * @param velocity Goal velocity
   */
  public void runVelocity(double upperSpeed, double lowerSpeed) {
    runVelocity(upperSpeed, 0, lowerSpeed, 0);
  }

  /**
   * Set goal velocity of the mechanism in degrees of elevation per second
   *
   * @param velocity Goal velocity
   */
  public void runVelocity(double upperSpeed, int upperSlot, double lowerSpeed, int lowerSlot) {
    upperSetpoint = upperSpeed;
    lowerSetpoint = lowerSpeed;
    controlMode = ControlMode.Velocity;
    upperIO.runVelocity(upperSpeed, upperSlot);
    lowerIO.runVelocity(lowerSpeed, lowerSlot);
  }

  /** Stop motor */
  public void stop() {
    controlMode = ControlMode.Neutral;
    upperIO.stop();
    lowerIO.stop();
  }

  /**
   * Velocity of the mechanism in degrees of elevation per second
   *
   * @return Velocity
   */
  public double getUpperVelocity() {
    return upperInputs.velocityRotsPerSec;
  }

  /**
   * Velocity of the mechanism in degrees of elevation per second
   *
   * @return Velocity
   */
  public double getLowerVelocity() {
    return lowerInputs.velocityRotsPerSec;
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
  public double getUpperTorqueCurrent() {
    return upperInputs.torqueCurrentAmps;
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
  public double getLowerTorqueCurrent() {
    return lowerInputs.torqueCurrentAmps;
  }

  /**
   * Check if mechanism is at goal speed with error of setpointBandSpeed
   *
   * @return True if in speed control mode and mechanism is at goal speed, false otherwise
   */
  @AutoLogOutput(key = "Shooter/AtBothSetpoints")
  public boolean atSetpoint() {
    return upperAtVelocity() && lowerAtVelocity();
  }

  private void setPID() {
    SlotConfigs[] newconfig = new SlotConfigs[kP.length];
    for (int i = 0; i < Math.min(newconfig.length, 3); i++) {
      newconfig[i] = new SlotConfigs().withKP(kP[i].get()).withKD(kD[i].get()).withKS(kS[i].get());
    }
    upperIO.setPID(newconfig);
    lowerIO.setPID(newconfig);
  }

  private void setMotionMagic() {
    MotionMagicConfigs newconfig = new MotionMagicConfigs().withMotionMagicAcceleration(mmA.get());
    upperIO.setMotionMagic(newconfig);
    lowerIO.setMotionMagic(newconfig);
  }

  @AutoLogOutput(key = "Shooter/Upper/AtSetpoint")
  private boolean upperAtVelocity() {
    switch (controlMode) {
      case Velocity:
        return Math.abs(upperSetpoint - upperInputs.velocityRotsPerSec) < setpointBandSpeed.get();
      default:
        return false;
    }
  }

  @AutoLogOutput(key = "Shooter/Lower/AtSetpoint")
  private boolean lowerAtVelocity() {
    switch (controlMode) {
      case Velocity:
        return Math.abs(lowerSetpoint - lowerInputs.velocityRotsPerSec) < setpointBandSpeed.get();
      default:
        return false;
    }
  }
}
