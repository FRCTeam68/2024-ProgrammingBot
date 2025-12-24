package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  static class WristIOInputs {
    // TODO: do we make this consistant (rotations) or leave it more human readable (degrees/meters
    // for elevators)
    public double elevationDeg = 0.0;
    public double velocityDegPerSec = 0.0;

    public boolean leaderConnected = false;
    public double leaderAppliedVoltage = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderTorqueCurrentAmps = 0.0;
    public double leaderTempCelsius = 0.0;

    public boolean followerConnected = false;
    public double followerAppliedVoltage = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTorqueCurrentAmps = 0.0;
    public double followerTempCelsius = 0.0;
  }

  default void updateInputs(WristIOInputs inputs) {}

  /** Run motor at volts */
  default void runVolts(double volts) {}

  /** Run mechanism at position */
  default void runPosition(double position, int slot) {}

  /** Stop motor */
  default void stop() {}

  /**
   * Set the current mechanism elevation
   *
   * @param elevation current elevation
   */
  default void setPosition(double elevation) {}

  /**
   * Set slot configs
   *
   * <p>Gravity type and static feedforward sign are ignored and use static values instead.
   *
   * <ul>
   *   <li><b>Available slots:</b> [0,2]
   * </ul>
   *
   * @param newConfig PID gains
   */
  public default void setPID(SlotConfigs... newConfig) {}

  /** Set motion magic velocity, acceleration and jerk. */
  public default void setMotionMagic(MotionMagicConfigs newconfig) {}

  /** Enables the software motion limits */
  public default void setSoftwareLimitsEnabled(boolean enabled) {}
}
