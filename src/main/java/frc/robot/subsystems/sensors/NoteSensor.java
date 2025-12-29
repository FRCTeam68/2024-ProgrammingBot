package frc.robot.subsystems.sensors;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class NoteSensor extends SubsystemBase {
  @Getter
  @Setter
  @AutoLogOutput(key = "NoteSensor/IsDetected")
  private boolean detected = false;

  private Supplier<Boolean> inputRaw;

  private DigitalInput inputDIO = new DigitalInput(0);
  private LoggedNetworkBoolean inputSim;

  // ----- solution attempt 1 ---------------
  private final Notifier fastLoop;
  private Debouncer inputDebouncer = new Debouncer(0.01, DebounceType.kRising);

  // ----- solution attempt 3 ----------------
  @Getter
  @Setter
  @AutoLogOutput(key = "NoteSensor/IsTripped")
  private boolean tripped = false;

  public Counter noteTriggerCnt;

  public NoteSensor() {
    switch (Constants.getMode()) {
      case REAL, REPLAY -> inputRaw = () -> !inputDIO.get();
      case SIM -> {
        inputSim = new LoggedNetworkBoolean("NoteSensor/SimulatedInput", true);
        inputRaw = inputSim::get;
      }
    }

    // ------ solution attempt 1  --------
    fastLoop = new Notifier(this::fastPeriodic);
    fastLoop.startPeriodic(0.005); // 5 ms = 200 Hz

    // ------ solution attempt 2  --------
    // every 5ms, sample for note sensor ant latch high if signal is high.
    // signal is only 20ms high when feeding in and passing by sensor.
    // super().addPeriodic(() -> {
    //   sampledValueHigh ? true : inputDebouncer.calculate(inputRaw.get());
    // }, 0.01, 0.005);

    // ------ solution attempt 3  --------
    AnalogInput input = new AnalogInput(0);
    input.setAverageBits(4);

    // Initializes an AnalogTrigger using the above input
    AnalogTrigger noteTriggerAnalog = new AnalogTrigger(input);

    // Sets the trigger to enable at a voltage of 4 volts, and disable at a value of 1.5 volts
    noteTriggerAnalog.setLimitsVoltage(1.5, 4);

    noteTriggerCnt = new Counter(noteTriggerAnalog);
    // above already sets calls setUpSource counter

    // Apr 4th, states, testing inverted logic level shifter
    noteTriggerCnt.setUpSourceEdge(false, true);
  }

  private void fastPeriodic() {
    // High‑rate control loop
    // sample for note sensor and latch high if signal is high.
    if (!detected) {
      if (inputDebouncer.calculate(inputRaw.get())) {
        detected = true;
      }
    }
  }

  @Override
  public void periodic() {

    // ------ solution attempt 3  --------
    int tmpNoteTrigCnt = noteTriggerCnt.get();
    if (tmpNoteTrigCnt > 0) {
      // expect rising edge and falling edge of pulse to be counted
      // Also, can only measure period if count is 2 or more
      // Period is measured from last 2 edges
      tripped = true;
      noteTriggerCnt.reset();
    }
    Logger.recordOutput("NoteSensor/TriggerCount", tmpNoteTrigCnt);
  }
}
