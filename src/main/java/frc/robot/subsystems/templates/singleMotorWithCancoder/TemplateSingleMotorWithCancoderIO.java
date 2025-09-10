package frc.robot.subsystems.templates.singleMotorWithCancoder;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import org.littletonrobotics.junction.AutoLog;

public interface TemplateSingleMotorWithCancoderIO {
  @AutoLog
  static class TemplateSinglemotorWithCancoderIOInputs {
    public double positionRotations = 0.0;
    public double velocityRotsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
    public boolean talonConnected = false;
    public boolean tempFault = false;
    public boolean cancoderConnected = false;
    public MagnetHealthValue magnetHealth = MagnetHealthValue.Magnet_Invalid;
  }

  default void updateInputs(TemplateSinglemotorWithCancoderIOInputs inputs) {}

  /** Run motor at volts */
  default void setVolts(double volts) {}

  /**
   * Run motor to speed
   *
   * @param speed Velocity in mechanism rotations per second
   * @param slot Slot must be within [0,2]
   */
  default void setSpeed(double speed, int slot) {}

  /**
   * Run motor to position
   *
   * @param position Position in mechanism rotations
   * @param slot Slot must be within [0,2]
   */
  default void setPosition(double position, int slot) {}

  /** Stop motor */
  default void stop() {}

  /**
   * Stop motor and set zero position
   *
   * @param offset Offset from current position in mechanism rotations
   */
  default void zero(double offset) {}

  /**
   * Set slot configs for closed loop control on drive motor. Maximum 3 slot configs. All additional
   * are ignored.
   */
  default void setPID(SlotConfigs... newconfig) {}

  /** Set motion magic velocity, acceleration, and jerk */
  default void setMotionMagic(MotionMagicConfigs newconfig) {}
}
