package frc.robot.subsystems.wrist;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;
import lombok.Getter;

public class WristIOReal implements WristIO {
  @Getter private static final double reduction = 10.0 * 7.0 * (64.0 / 18.0);
  private final GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;
  private final StaticFeedforwardSignValue feedforwardSign =
      StaticFeedforwardSignValue.UseClosedLoopSign;

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
  private final StatusSignal<Voltage> followerAppliedVoltage;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Current> followerTorqueCurrent;
  private final StatusSignal<Temperature> followerTempCelsius;

  // Control requests
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final PositionVoltage positionOut = new PositionVoltage(0).withEnableFOC(true);
  private final NeutralOut neutralOut = new NeutralOut();

  public WristIOReal() {
    // Hardware
    talon = new TalonFX(32, new CANBus("rio"));
    followerTalon = new TalonFX(33, new CANBus("rio"));
    followerTalon.setControl(new Follower(talon.getDeviceID(), true));

    // Configure motor
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
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(Wrist.getMaximum());
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(Wrist.getMinimum());
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
        followerAppliedVoltage,
        followerSupplyCurrent,
        followerTorqueCurrent,
        followerTempCelsius);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.elevationDeg = Units.rotationsToDegrees(leaderPosition.getValueAsDouble());
    inputs.velocityDegPerSec = Units.rotationsToDegrees(leaderVelocity.getValueAsDouble());

    inputs.leaderConnected =
        BaseStatusSignal.isAllGood(
            leaderPosition,
            leaderVelocity,
            leaderAppliedVoltage,
            leaderSupplyCurrent,
            leaderTorqueCurrent);
    inputs.leaderAppliedVoltage = leaderAppliedVoltage.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.leaderTorqueCurrentAmps = leaderTorqueCurrent.getValueAsDouble();
    inputs.leaderTempCelsius = leaderTempCelsius.getValueAsDouble();

    inputs.followerConnected =
        BaseStatusSignal.isAllGood(
            followerAppliedVoltage, followerSupplyCurrent, followerTorqueCurrent);
    inputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
    inputs.followerTempCelsius = followerTempCelsius.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void runPosition(double elevation) {
    talon.setControl(positionOut.withPosition(Units.degreesToRotations(elevation)));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setPosition(double elevation) {
    talon.setPosition(Units.degreesToRotations(elevation));
  }

  @Override
  public void setPID(SlotConfigs newConfig) {
    config.Slot0 =
        Slot0Configs.from(
            newConfig.withGravityType(gravityType).withStaticFeedforwardSign(feedforwardSign));

    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void setSoftwareLimitsEnabled(boolean enabled) {
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = enabled;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = enabled;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
  }
}
