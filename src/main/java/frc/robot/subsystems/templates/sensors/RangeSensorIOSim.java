package frc.robot.subsystems.templates.sensors;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class RangeSensorIOSim implements RangeSensorIO {
  private final LoggedNetworkBoolean detected;
  private final LoggedNetworkNumber distance_mm;

  public RangeSensorIOSim(String name) {
    detected = new LoggedNetworkBoolean("/Simulation/" + name + " Range Sensor/Detected", false);
    distance_mm = new LoggedNetworkNumber("/Simulation/" + name + " Range Sensor/Distance mm", 0.0);
  }

  @Override
  public void updateInputs(RangeSystemIOInputs inputs) {
    inputs.connected = true;
    inputs.detected = detected.get();
    inputs.distance_mm = distance_mm.get();
  }
}
