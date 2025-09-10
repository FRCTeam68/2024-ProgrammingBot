package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
  @AutoLog
  static class RollerSystemIOInputs {
    public double positionRotations = 0.0;
    public double velocityRotsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public boolean connected = false;
    public boolean tempFault = false;
  }

  default void updateInputs(RollerSystemIOInputs inputs) {}

  /** Run roller at volts */
  default void setVolts(double volts) {}

  /**
   * Run roller to speed
   *
   * @param speed Velocity in mechanism rotations per second
   * @param slot Slot must be within [0,2]
   */
  default void setSpeed(double speed, int slot) {}

  /**
   * Run roller to position
   *
   * @param rotations Position in mechanism rotations
   * @param slot Slot must be within [0,2]
   */
  default void setPosition(double rotations, int slot) {}

  /** Stop roller */
  default void stop() {}

  /** Stop roller and set current position to zero */
  default void zero() {}

  /**
   * Set slot configs for closed loop control on drive motor. Maximum 3 slot configs. All additional
   * are ignored.
   */
  default void setPID(SlotConfigs... newconfig) {}

  /** Set motion magic velocity, acceleration, and jerk */
  default void setMotionMagic(MotionMagicConfigs newconfig) {}
}
