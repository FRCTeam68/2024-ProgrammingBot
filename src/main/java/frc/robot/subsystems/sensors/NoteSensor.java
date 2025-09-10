package frc.robot.subsystems.sensors;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class NoteSensor extends SubsystemBase {
  @Getter
  @AutoLogOutput(key = "NoteSensor/HaveNote")
  private boolean haveNote = false;

  DigitalInput inputDIO = new DigitalInput(0);
  LoggedNetworkBoolean inputSim;
  @Getter LoggedNetworkBoolean automaticNoteSim;

  Debouncer inputDebouncer = new Debouncer(0.06, DebounceType.kRising);

  public NoteSensor() {
    if (Constants.getMode() == Mode.SIM) {
      inputSim = new LoggedNetworkBoolean("NoteSensor/SimulatedInput", false);
      automaticNoteSim = new LoggedNetworkBoolean("NoteSensor/AutomaticSimulatedInput", false);
    }
  }

  public void periodic() {
    // TODO: code in automatic simulated note
    switch (Constants.getMode()) {
      case REAL, REPLAY:
        Logger.recordOutput("NoteSensor/SensorInput", inputDIO.get());
        haveNote = inputDebouncer.calculate(inputDIO.get());
        break;

      case SIM:
        Logger.recordOutput("NoteSensor/SensorInput", inputSim.get());
        haveNote = inputDebouncer.calculate(inputSim.get());
        break;
    }
  }

  public Command automaticSimulatedNote(double delaySeconds) {
    // if (automaticNoteSim.get()) {
    return Commands.sequence(
        Commands.waitSeconds(delaySeconds), Commands.runOnce(() -> inputSim.set(true)));
    // } else {
    //   return Commands.none();
    // }
  }
}
