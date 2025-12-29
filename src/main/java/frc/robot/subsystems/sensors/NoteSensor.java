package frc.robot.subsystems.sensors;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;

public class NoteSensor extends SubsystemBase {

  private final Notifier fastLoop;

  // ----- solution attempt 1 ---------------
  private DigitalInput inputDIO;
  private Debouncer inputDebouncer;

  @Getter
  @Setter
  @AutoLogOutput(key = "NoteSensor/IsDetected")
  private boolean detected = false;

  // ----- solution attempt 2 ----------------
  private AnalogInput inputAIO;
  private AnalogTrigger noteTriggerAnalog;

  @Getter
  @Setter
  @AutoLogOutput(key = "NoteSensor/IsTripped")
  private boolean tripped = false;

  public NoteSensor() {

    fastLoop = new Notifier(this::fastPeriodic);
    fastLoop.startPeriodic(0.005); // 5 ms = 200 Hz

    // ------ solution attempt 1  --------
    inputDIO = new DigitalInput(0);
    inputDebouncer = new Debouncer(0.008, DebounceType.kRising);

    // ------ solution attempt 2  --------
    inputAIO = new AnalogInput(0);
    inputAIO.setAverageBits(4);
    noteTriggerAnalog = new AnalogTrigger(inputAIO);
    // Sets the trigger to be high above 4 volts, and low at below 1.5 volts
    noteTriggerAnalog.setLimitsVoltage(1.5, 4);
  }

  private void fastPeriodic() {
    // High‑rate control loop
    // sample for note sensor.  remember low pulse means detection
    detected = !inputDebouncer.calculate(inputDIO.get());
    tripped = !noteTriggerAnalog.getTriggerState();
  }

  @Override
  public void periodic() {}
}
