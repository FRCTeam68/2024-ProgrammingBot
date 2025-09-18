package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PhoenixUtil.ControlMode;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends SubsystemBase {
  private final String name;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();

  private final Alert disconnectedAlert;
  private final Alert tempAlert;

  private LoggedTunableNumber rollerkP;
  private LoggedTunableNumber rollerkD;
  private LoggedTunableNumber rollerkS;
  private LoggedTunableNumber rollerMMV;
  private LoggedTunableNumber rollerMMA;

  @Getter private double setpoint = 0.0;

  @Getter private ControlMode mode = ControlMode.Neutral;

  public RollerSystem(String name, RollerSystemIO io) {
    this.name = name;
    this.io = io;

    rollerkP = new LoggedTunableNumber(name + "/kP");
    rollerkD = new LoggedTunableNumber(name + "/kD");
    rollerkS = new LoggedTunableNumber(name + "/kS");
    rollerMMV = new LoggedTunableNumber(name + "/MMV");
    rollerMMA = new LoggedTunableNumber(name + "/MMA");

    disconnectedAlert = new Alert(name + " motor disconnected!", AlertType.kError);
    tempAlert = new Alert(name + "motor is too hot!", AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnectedAlert.set(!inputs.connected);
    tempAlert.set(inputs.tempCelsius > Constants.warningTemp);

    Logger.recordOutput(name + "/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput(name + "/SetpointRotPerSec", (mode == ControlMode.Speed) ? setpoint : 0);
    Logger.recordOutput(
        name + "/SetpointPositionDeg", (mode == ControlMode.Position) ? setpoint : 0);

    // Update tunable numbers
    if (Constants.tuningMode) {
      if (rollerkP.hasChanged(hashCode())
          || rollerkD.hasChanged(hashCode())
          || rollerkS.hasChanged(hashCode())) {
        io.setPID(
            new SlotConfigs()
                .withKP(rollerkP.getAsDouble())
                .withKD(rollerkD.getAsDouble())
                .withKS(rollerkS.getAsDouble()));
      }
      if (rollerMMV.hasChanged(hashCode()) || rollerMMA.hasChanged(hashCode())) {
        io.setMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(rollerMMV.getAsDouble())
                .withMotionMagicAcceleration(rollerMMA.getAsDouble()));
      }
    }
  }

  /** Must call this once and only once in robotcontainer after each RollerSystem is created */
  public void setPID(SlotConfigs newConfig) {
    rollerkP.initDefault(newConfig.kP);
    rollerkD.initDefault(newConfig.kD);
    rollerkS.initDefault(newConfig.kS);
    io.setPID(newConfig);
  }

  /** Must call this once and only once in robotcontainer after each RollerSystem is created */
  public void setMotionMagic(MotionMagicConfigs newconfig) {
    rollerMMV.initDefault(newconfig.MotionMagicCruiseVelocity);
    rollerMMA.initDefault(newconfig.MotionMagicAcceleration);
    io.setMotionMagic(newconfig);
  }

  /** Run roller at volts */
  public void setVolts(double inputVolts) {
    setpoint = inputVolts;
    mode = ControlMode.Voltage;
    io.setVolts(inputVolts);
  }

  /**
   * Run roller to speed
   *
   * @param speed Velocity in mechanism rotations per second
   */
  public void setSpeed(double speed) {
    setpoint = speed;
    mode = ControlMode.Speed;
    io.setSpeed(speed, 0);
  }

  /**
   * Run roller to position
   *
   * @param rotations Position in mechanism rotations
   */
  public void setPosition(double position) {
    setpoint = position;
    mode = ControlMode.Position;
    io.setPosition(position, 0);
  }

  /** Stop roller */
  public void stop() {
    mode = ControlMode.Neutral;
    io.stop();
  }

  /** Stop roller and set current position to zero */
  public void zero() {
    io.zero();
  }

  /**
   * @return Velocity of roller in mechanism rotations per second
   */
  public double getSpeed() {
    return inputs.velocityRotsPerSec;
  }

  /**
   * @return Position of roller in mechanism rotations
   */
  public double getPosition() {
    return inputs.positionRotations;
  }

  /**
   * @return TorqueCurrent of roller
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }
}
