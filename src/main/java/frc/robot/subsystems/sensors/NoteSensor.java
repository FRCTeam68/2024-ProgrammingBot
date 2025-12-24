package frc.robot.subsystems.sensors;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class NoteSensor extends SubsystemBase {
  @Getter
  @AutoLogOutput(key = "NoteSensor/IsDetected")
  private boolean detected = false;

  private Supplier<Boolean> inputRaw;

  private DigitalInput inputDIO = new DigitalInput(0);
  private LoggedNetworkBoolean inputSim;

  private Debouncer inputDebouncer = new Debouncer(0.01, DebounceType.kRising);

  public NoteSensor() {
    switch (Constants.getMode()) {
      case REAL, REPLAY -> inputRaw = () -> !inputDIO.get();
      case SIM -> {
        inputSim = new LoggedNetworkBoolean("NoteSensor/SimulatedInput", true);
        inputRaw = inputSim::get;
      }
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("NoteSensor/RawSensorInput", inputRaw.get());
    detected = inputDebouncer.calculate(inputRaw.get());
  }
}
