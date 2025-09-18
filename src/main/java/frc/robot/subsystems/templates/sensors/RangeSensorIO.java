package frc.robot.subsystems.templates.sensors;

import org.littletonrobotics.junction.AutoLog;

public interface RangeSensorIO {
  @AutoLog
  static class RangeSystemIOInputs {
    public boolean connected = false;
    public boolean detected = false;
    public double distance_mm = 0.0;
  }

  default void updateInputs(RangeSystemIOInputs inputs) {}
}
