package frc.robot.subsystems.templates.singleMotorWithCancoder;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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

/* Items to be Configured:
  reduction
  gravity type
  CAN id's
  CAN bus's
  inverted value
  neutral mode
  current limits
  software limits
  feedback
  cancoder configuration
*/

public class TemplateSingleMotorWithCancoderIOTalonFX implements TemplateSingleMotorWithCancoderIO {
  @Getter private static final double rotorToSensorReduction = 1;
  @Getter private static final double sensorToMechanismReduction = 1;
  private final GravityTypeValue gravityType = GravityTypeValue.Arm_Cosine;

  // Hardware
  private final TalonFX talon;
  private final CANcoder cancoder;

  // Configuration
  private final TalonFXConfiguration talonConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;
  private final StatusSignal<MagnetHealthValue> magnetHealth;

  private final Debouncer talonConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);
  private final Debouncer cancoderConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  // Control requests
  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
  private final MotionMagicVelocityTorqueCurrentFOC velocityOut =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final MotionMagicTorqueCurrentFOC positionOut = new MotionMagicTorqueCurrentFOC(0);
  private final NeutralOut neutralOut = new NeutralOut();

  public TemplateSingleMotorWithCancoderIOTalonFX() {
    talon = new TalonFX(0, "rio");
    cancoder = new CANcoder(0, "rio");

    // Configure Motor
    talonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Current limits
    talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfig.CurrentLimits.StatorCurrentLimit = 120;
    talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfig.CurrentLimits.SupplyCurrentLimit = 70;
    talonConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    talonConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
    // Motion limits
    talonConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5.4;
    talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    // Feedback
    talonConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
    talonConfig.Feedback.RotorToSensorRatio = rotorToSensorReduction;
    talonConfig.Feedback.SensorToMechanismRatio = sensorToMechanismReduction;
    // CANcoder
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = 0.0;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));
    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));

    // Configure status signals
    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    tempCelsius = talon.getDeviceTemp();
    magnetHealth = cancoder.getMagnetHealth();

    tryUntilOk(
        5,
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                position,
                velocity,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                magnetHealth));
    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(4, tempCelsius));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(talon, cancoder));
    PhoenixUtil.registerSignals(
        false,
        position,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius,
        magnetHealth);
  }

  @Override
  public void updateInputs(TemplateSinglemotorWithCancoderIOInputs inputs) {
    inputs.talonConnected =
        talonConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent));
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRotsPerSec = velocity.getValueAsDouble();
    inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();

    inputs.cancoderConnected =
        cancoderConnectedDebouncer.calculate(BaseStatusSignal.isAllGood(magnetHealth));
    inputs.magnetHealth = magnetHealth.getValue();
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
        case 0 -> talonConfig.Slot0 = Slot0Configs.from(slotConfig);
        case 1 -> talonConfig.Slot1 = Slot1Configs.from(slotConfig);
        case 2 -> talonConfig.Slot2 = Slot2Configs.from(slotConfig);
      }
    }
    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
  }

  @Override
  public void setMotionMagic(MotionMagicConfigs newconfig) {
    talonConfig.MotionMagic.MotionMagicCruiseVelocity = newconfig.MotionMagicCruiseVelocity;
    talonConfig.MotionMagic.MotionMagicAcceleration = newconfig.MotionMagicAcceleration;
    talonConfig.MotionMagic.MotionMagicJerk = newconfig.MotionMagicJerk;
    tryUntilOk(5, () -> talon.getConfigurator().apply(talonConfig, 0.25));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void zero(double offset) {
    talon.setPosition(offset);
  }
}
