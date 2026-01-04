package frc.robot.subsystems.wrist;

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

  /**
   * Run mechanism at volts.
   *
   * @param volts Voltage
   */
  default void runVolts(double volts) {}

  /**
   * Run mechanism to elevation.
   *
   * @param elevation Elevation in degrees
   */
  default void runPosition(double elevation) {}

  /** Stop mechanism */
  default void stop() {}

  /**
   * Set current mechanism elevation.
   *
   * @param elevation Elevation in degrees
   */
  default void setPosition(double elevation) {}

  /**
   * Set PID gains.
   *
   * <p>Gravity type and static feedforward sign are ignored and use static values instead.
   *
   * @param newConfig PID gains
   */
  public default void setPID(SlotConfigs newConfig) {}

  /** Enables the software motion limits */
  public default void setSoftwareLimitsEnabled(boolean enabled) {}
}
