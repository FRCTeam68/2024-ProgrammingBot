package frc.robot.subsystems.templates.sensors;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class RangeSensor extends SubsystemBase {
  private final RangeSensorIO io;
  private final String name;
  protected final RangeSystemIOInputsAutoLogged inputs = new RangeSystemIOInputsAutoLogged();

  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Alert disconnectedAlert;

  public RangeSensor(String name, RangeSensorIO io) {
    this.io = io;
    this.name = name;

    disconnectedAlert = new Alert(name + " sensor disconnected.", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Sensors/" + name, inputs);
    disconnectedAlert.set(!connectedDebouncer.calculate(inputs.connected));
  }

  public boolean isDetected() {
    return inputs.detected;
  }

  public double getDistance_mm() {
    return inputs.distance_mm;
  }
}
