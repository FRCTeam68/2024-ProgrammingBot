package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import lombok.Getter;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class ShooterIOReal implements ShooterIO {
  @Getter private static final double reduction = 1.0;

  // Hardware
  private final TalonFX talon;

  // Configuration
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  // Control requests
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final VelocityVoltage velocityOut = new VelocityVoltage(0).withEnableFOC(true);
  private final NeutralOut neutralOut = new NeutralOut();

  public ShooterIOReal(int id, InvertedValue invertedValue) {
    talon = new TalonFX(id, "rio");

    // Configure Motor
    config.MotorOutput.Inverted = invertedValue;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // Current limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    // Feedback
    config.Feedback.SensorToMechanismRatio = reduction;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Configure status signals
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent));
    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(4, tempCelsius));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon));
    PhoenixUtil.registerSignals(
        false, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, tempCelsius);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.isAllGood(
            position, velocity, appliedVoltage, supplyCurrent, torqueCurrent);
    inputs.positionRots = position.getValueAsDouble();
    inputs.velocityRotsPerSec = velocity.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runVelocity(double velocity, int slot) {
    talon.setControl(velocityOut.withVelocity(velocity).withSlot(slot));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setPosition(double rotations) {
    talon.setPosition(rotations);
  }

  @Override
  public void setPID(SlotConfigs... newConfig) {
    for (int i = 0; i < Math.min(newConfig.length, 3); i++) {
      SlotConfigs slotConfig = newConfig[i];
      switch (i) {
        case 0 -> config.Slot0 = Slot0Configs.from(slotConfig);
        case 1 -> config.Slot1 = Slot1Configs.from(slotConfig);
        case 2 -> config.Slot2 = Slot2Configs.from(slotConfig);
      }
    }
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setMotionMagic(MotionMagicConfigs newConfig) {
    tryUntilOk(5, () -> talon.getConfigurator().apply(newConfig, 0.25));
  }
}
