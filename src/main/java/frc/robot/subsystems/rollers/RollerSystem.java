package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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

  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Alert disconnectedAlert;
  private final Alert tempAlert;

  private LoggedTunableNumber rollerkP;
  private LoggedTunableNumber rollerkD;
  private LoggedTunableNumber rollerkS;

  @Getter private double setpoint = 0.0;

  @Getter private ControlMode mode = ControlMode.Neutral;

  public RollerSystem(String name, RollerSystemIO io) {
    this.name = name;
    this.io = io;

    rollerkP = new LoggedTunableNumber(name + "/kP");
    rollerkD = new LoggedTunableNumber(name + "/kD");
    rollerkS = new LoggedTunableNumber(name + "/kS");

    disconnectedAlert = new Alert(name + " motor disconnected!", AlertType.kError);
    tempAlert = new Alert(name + "motor is too hot!", AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnectedAlert.set(!connectedDebouncer.calculate(inputs.connected));
    tempAlert.set(inputs.tempCelsius > Constants.warningTempCelsius);

    Logger.recordOutput(name + "/SetpointVolts", (mode == ControlMode.Voltage) ? setpoint : 0);
    Logger.recordOutput(name + "/SetpointRotPerSec", (mode == ControlMode.Velocity) ? setpoint : 0);
    Logger.recordOutput(
        name + "/SetpointPositionDeg", (mode == ControlMode.Position) ? setpoint : 0);

    // TODO: do we always check this. we wouldn't need to call set pid in rollersystem
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
    }
  }

  /** Must call this once and only once in robotcontainer after each RollerSystem is created */
  public void setPID(SlotConfigs newConfig) {
    rollerkP.initDefault(newConfig.kP);
    rollerkD.initDefault(newConfig.kD);
    rollerkS.initDefault(newConfig.kS);
    io.setPID(newConfig);
  }

  /** Run roller at volts */
  public void setVolts(double inputVolts) {
    setpoint = inputVolts;
    mode = ControlMode.Voltage;
    io.setVolts(inputVolts);
  }

  /**
   * Run roller at velocity
   *
   * @param velocity Velocity in mechanism rotations per second
   */
  public void setVelocity(double velocity) {
    setpoint = velocity;
    mode = ControlMode.Velocity;
    io.setVelocity(velocity, 0);
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
  public double getVelocity() {
    return inputs.velocityRotsPerSec;
  }

  /**
   * @return Position of roller in mechanism rotations
   */
  public double getPosition() {
    return inputs.positionRots;
  }

  /**
   * @return TorqueCurrent of roller
   */
  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }
}
