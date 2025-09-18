package frc.robot.subsystems.Devbot.wrist;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import lombok.Getter;

/** Generic roller IO implementation for a roller or series of rollers using a Kraken. */
public class WristIOTalonFX implements WristIO {
  @Getter private static final double reduction = 5 * (64 / 18);
  private final GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;

  // Hardware
  private final TalonFX talon;
  private final TalonFX followerTalon;

  // Configuration
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  // Status signals
  private final StatusSignal<Angle> leaderPosition;
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVoltage;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Current> leaderTorqueCurrent;
  private final StatusSignal<Temperature> leaderTempCelsius;
  private final StatusSignal<Angle> followerPosition;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Temperature> followerTempCelsius;

  private final Debouncer connectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer followerConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  // Control requests
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVelocityTorqueCurrentFOC velocityOut =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC positionOut = new MotionMagicTorqueCurrentFOC(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public WristIOTalonFX() {
    talon = new TalonFX(33, "rio");
    followerTalon = new TalonFX(32, "rio");
    followerTalon.setControl(new Follower(talon.getDeviceID(), true));

    // Configure Motor
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Current limits
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 70;
    config.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.CurrentLimits.SupplyCurrentLowerTime = 1;
    // Motion limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5.4;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    // Feedback
    config.Feedback.SensorToMechanismRatio = reduction;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // Configure status signals
    leaderPosition = talon.getPosition();
    leaderVelocity = talon.getVelocity();
    leaderAppliedVoltage = talon.getMotorVoltage();
    leaderSupplyCurrent = talon.getSupplyCurrent();
    leaderTorqueCurrent = talon.getTorqueCurrent();
    leaderTempCelsius = talon.getDeviceTemp();
    followerPosition = followerTalon.getPosition();
    followerVelocity = followerTalon.getVelocity();
    followerAppliedVoltage = followerTalon.getMotorVoltage();
    followerSupplyCurrent = followerTalon.getSupplyCurrent();
    followerTorqueCurrent = followerTalon.getTorqueCurrent();
    followerTempCelsius = followerTalon.getDeviceTemp();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                leaderPosition,
                leaderVelocity,
                leaderAppliedVoltage,
                leaderSupplyCurrent,
                leaderTorqueCurrent,
                followerPosition,
                followerVelocity,
                followerAppliedVoltage,
                followerSupplyCurrent,
                followerTorqueCurrent));
    tryUntilOk(
        5,
        () -> BaseStatusSignal.setUpdateFrequencyForAll(4, leaderTempCelsius, followerTempCelsius));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon, followerTalon));
    PhoenixUtil.registerSignals(
        false,
        leaderPosition,
        leaderVelocity,
        leaderAppliedVoltage,
        leaderSupplyCurrent,
        leaderTorqueCurrent,
        leaderTempCelsius,
        followerPosition,
        followerVelocity,
        followerAppliedVoltage,
        followerSupplyCurrent,
        followerTorqueCurrent,
        followerTempCelsius);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.elevationDeg = leaderPosition.getValueAsDouble();
    inputs.elevationDeg = followerPosition.getValueAsDouble() - leaderPosition.getValueAsDouble();
    inputs.velocityDegPerSec = leaderVelocity.getValueAsDouble() * 360;
    inputs.velocityOffsetDegPerSec =
        (followerVelocity.getValueAsDouble() - leaderVelocity.getValueAsDouble()) * 360;

    inputs.leaderConnected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                leaderPosition,
                leaderVelocity,
                leaderAppliedVoltage,
                leaderSupplyCurrent,
                leaderTorqueCurrent));
    inputs.leaderAppliedVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.leaderTorqueCurrentAmps = leaderTorqueCurrent.getValueAsDouble();
    inputs.leaderTempCelsius = leaderTempCelsius.getValueAsDouble();

    inputs.followerConnected =
        followerConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                followerPosition,
                followerVelocity,
                followerAppliedVoltage,
                followerSupplyCurrent,
                followerTorqueCurrent));
    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
    inputs.followerTempCelsius = followerTempCelsius.getValueAsDouble();
  }

  @Override
  public void setVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setSpeed(double speed, int slot) {
    talon.setControl(velocityOut.withVelocity(speed).withSlot(slot));
  }

  @Override
  public void setPosition(double position, int slot) {
    talon.setControl(positionOut.withPosition(position).withSlot(slot));
  }

  @Override
  public void setPID(SlotConfigs... newConfig) {
    for (int i = 0; i < Math.min(newConfig.length, 3); i++) {
      SlotConfigs slotConfig =
          new SlotConfigs()
              .withGravityType(gravityType)
              .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
              .withKP(newConfig[i].kP)
              .withKI(newConfig[i].kI)
              .withKD(newConfig[i].kD)
              .withKV(newConfig[i].kV)
              .withKA(newConfig[i].kA)
              .withKG(newConfig[i].kG)
              .withKS(newConfig[i].kS);

      switch (i) {
        case 0 -> config.Slot0 = Slot0Configs.from(slotConfig);
        case 1 -> config.Slot1 = Slot1Configs.from(slotConfig);
        case 2 -> config.Slot2 = Slot2Configs.from(slotConfig);
      }
    }
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setMotionMagic(MotionMagicConfigs newconfig) {
    config.MotionMagic.MotionMagicCruiseVelocity = newconfig.MotionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = newconfig.MotionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = newconfig.MotionMagicJerk;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void zero(double offset) {
    setVolts(0);
    talon.setPosition(offset);
  }
}
