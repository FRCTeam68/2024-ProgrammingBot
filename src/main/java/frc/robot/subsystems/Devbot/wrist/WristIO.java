package frc.robot.subsystems.Devbot.wrist;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  static class WristIOInputs {
    public double elevationDeg = 0.0;
    public double elevationOffsetDeg = 0.0;
    public double velocityDegPerSec = 0.0;
    public double velocityOffsetDegPerSec = 0.0;

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
  default void setVolts(double volts) {}

  /** Run mechanism at speed */
  default void setSpeed(double speed, int slot) {}

  /** Run mechanism at position */
  default void setPosition(double position, int slot) {}

  /** Stop motor */
  default void stop() {}

  /** Zero mechanism */
  default void zero(double offset) {}

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
}
