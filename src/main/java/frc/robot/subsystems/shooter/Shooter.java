package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CanBus;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.rollers.RollerSystemIOInputsAutoLogged;
import frc.robot.subsystems.rollers.RollerSystemIOSim;
import frc.robot.subsystems.rollers.RollerSystemIOTalonFX;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final double reduction = 1;
  private final int currentLimitAmps = 70;
  private final double moi = 0.1;

  private final RollerSystemIO upperIO;
  private final RollerSystemIO lowerIO;
  protected final RollerSystemIOInputsAutoLogged upperInputs = new RollerSystemIOInputsAutoLogged();
  protected final RollerSystemIOInputsAutoLogged lowerInputs = new RollerSystemIOInputsAutoLogged();

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
    new LoggedTunableNumber(pidKey + "0/kP", 5), new LoggedTunableNumber(pidKey + "1/kP", 5)
  };
  private LoggedTunableNumber[] kD = {
    new LoggedTunableNumber(pidKey + "0/kD", 0), new LoggedTunableNumber(pidKey + "1/kD", 0)
  };
  private LoggedTunableNumber[] kS = {
    new LoggedTunableNumber(pidKey + "0/kS", 0.5), new LoggedTunableNumber(pidKey + "1/kS", 0.5)
  };

  private LoggedTunableNumber mmV = new LoggedTunableNumber("Shooter/MotionMagic/Velocity", 40);
  private LoggedTunableNumber mmA =
      new LoggedTunableNumber("Shooter/MotionMagic/Acceleration", 120);

  private LoggedTunableNumber setpointBandSpeed =
      new LoggedTunableNumber("Shooter/SetpointBandSpeed", 0.2);

  @Getter private double upperSetpoint = 0.0;
  @Getter private double lowerSetpoint = 0.0;

  @Getter private ControlMode controlMode = ControlMode.Neutral;

  public Shooter() {
    // TODO: should this be defined here or in robot container
    switch (Constants.getMode()) {
      case REAL:
        upperIO =
            new RollerSystemIOTalonFX(
                21,
                CanBus.rio,
                currentLimitAmps,
                InvertedValue.Clockwise_Positive,
                NeutralModeValue.Coast,
                reduction);
        lowerIO =
            new RollerSystemIOTalonFX(
                20,
                CanBus.rio,
                currentLimitAmps,
                InvertedValue.CounterClockwise_Positive,
                NeutralModeValue.Coast,
                reduction);
        break;

      case SIM:
        upperIO = new RollerSystemIOSim(DCMotor.getFalcon500Foc(1), reduction, moi);
        lowerIO = new RollerSystemIOSim(DCMotor.getFalcon500Foc(1), reduction, moi);
        break;

      default:
        upperIO = new RollerSystemIO() {};
        lowerIO = new RollerSystemIO() {};
        break;
    }

    setPID();
    setMotionMagic();
  }

  public void periodic() {
    upperIO.updateInputs(upperInputs);
    lowerIO.updateInputs(lowerInputs);
    Logger.processInputs("Shooter/Upper", upperInputs);
    Logger.processInputs("Shooter/Lower", lowerInputs);
    upperDisconnectedAlert.set(!upperInputs.connected);
    lowerDisconnectedAlert.set(!lowerInputs.connected);
    upperTempAlert.set(upperInputs.tempFault);
    lowerTempAlert.set(lowerInputs.tempFault);

    Logger.recordOutput(
        "Shooter/Upper/SetpointVolts", (controlMode == ControlMode.Voltage) ? upperSetpoint : 0);
    Logger.recordOutput(
        "Shooter/Upper/SetpointRotsPerSec", (controlMode == ControlMode.Speed) ? upperSetpoint : 0);
    Logger.recordOutput(
        "Shooter/Lower/SetpointVolts", (controlMode == ControlMode.Voltage) ? lowerSetpoint : 0);
    Logger.recordOutput(
        "Shooter/Lower/SetpointRotsPerSec", (controlMode == ControlMode.Speed) ? lowerSetpoint : 0);

    Logger.recordOutput("Shooter/AtSetpoint", atSetpoint());

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

      if (mmV.hasChanged(hashCode()) || mmA.hasChanged(hashCode())) {
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
  public void setVolts(double upperVolts, double lowerVolts) {
    upperSetpoint = upperVolts;
    lowerSetpoint = lowerVolts;
    controlMode = ControlMode.Voltage;
    upperIO.setVolts(upperVolts);
    lowerIO.setVolts(lowerVolts);
  }

  /**
   * Set goal speed of the mechanism in degrees of elevation per second
   *
   * @param speed Goal speed
   */
  public void setSpeed(double upperSpeed, double lowerSpeed, int slot) {
    upperSetpoint = upperSpeed;
    lowerSetpoint = lowerSpeed;
    controlMode = ControlMode.Speed;
    upperIO.setSpeed(upperSpeed, slot);
    lowerIO.setSpeed(lowerSpeed, slot);
  }

  /** Stop motor */
  public void stop() {
    controlMode = ControlMode.Neutral;
    upperIO.stop();
    lowerIO.stop();
  }

  private void setPID() {
    SlotConfigs[] newconfig = new SlotConfigs[kP.length];
    for (int i = 0; i < Math.min(newconfig.length, 3); i++) {
      newconfig[i] =
          new SlotConfigs()
              .withKP(kP[i].getAsDouble())
              .withKD(kD[i].getAsDouble())
              .withKS(kS[i].getAsDouble());
    }
    upperIO.setPID(newconfig);
    lowerIO.setPID(newconfig);
  }

  private void setMotionMagic() {
    MotionMagicConfigs newconfig =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(mmV.getAsDouble())
            .withMotionMagicAcceleration(mmA.getAsDouble());
    upperIO.setMotionMagic(newconfig);
    lowerIO.setMotionMagic(newconfig);
  }

  /**
   * Velocity of the mechanism in degrees of elevation per second
   *
   * @return Velocity
   */
  public double getUpperSpeed() {
    return upperInputs.velocityRotsPerSec;
  }

  /**
   * Velocity of the mechanism in degrees of elevation per second
   *
   * @return Velocity
   */
  public double getLowerSpeed() {
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
  public boolean atSetpoint() {
    switch (controlMode) {
      case Speed:
        return (Math.abs(upperSetpoint - upperInputs.velocityRotsPerSec)
                < setpointBandSpeed.getAsDouble()
            && Math.abs(lowerSetpoint - lowerInputs.velocityRotsPerSec)
                < setpointBandSpeed.getAsDouble());
      default:
        return false;
    }
  }
}
