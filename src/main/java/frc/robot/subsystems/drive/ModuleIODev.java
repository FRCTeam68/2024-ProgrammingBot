package frc.robot.subsystems.drive;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.DriveConstants.ModuleConfig;
import frc.robot.util.PhoenixUtil;
import java.util.Queue;

public class ModuleIODev implements ModuleIO {
  private static final double driveCurrentLimitAmps = 80;
  private static final double turnCurrentLimitAmps = 40;
  public static final double driveReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double turnReduction = (150.0 / 7.0);

  // Hardware objects
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder cancoder;

  // Configurations
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  // Control requests
  // TODO: MA uses update freq 0. investigate
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Current> driveTorqueCurrent;
  private final StatusSignal<Temperature> driveTempCelsius;
  private final StatusSignal<Boolean> driveTempFault;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<MagnetHealthValue> turnMagnetHealth;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnSupplyCurrent;
  private final StatusSignal<Current> turnTorqueCurrent;
  private final StatusSignal<Temperature> turnTempCelsius;
  private final StatusSignal<Boolean> turnTempFault;
  private final StatusSignal<Boolean> turnEncoderSyncStickyFault;

  public ModuleIODev(ModuleConfig constants) {
    driveTalon = new TalonFX(constants.driveMotorId(), DriveConstants.canbus.getName());
    turnTalon = new TalonFX(constants.turnMotorId(), DriveConstants.canbus.getName());
    cancoder = new CANcoder(constants.encoderId(), DriveConstants.canbus.getName());

    // Configure drive motor
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    driveConfig.Feedback.SensorToMechanismRatio = driveReduction;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveCurrentLimitAmps;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -driveCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimit = driveCurrentLimitAmps;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // TODO: mechanical advantage had this, but I don't know if we want it
    driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    turnConfig.Feedback.FeedbackRemoteSensorID = constants.encoderId();
    turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnConfig.Feedback.RotorToSensorRatio = turnReduction;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = turnCurrentLimitAmps;
    turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -turnCurrentLimitAmps;
    turnConfig.CurrentLimits.StatorCurrentLimit = turnCurrentLimitAmps;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    // TODO: do we want these?
    // turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / DriveConstants.turnGearRatio;
    // turnConfig.MotionMagic.MotionMagicAcceleration =
    //     turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    // turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * DriveConstants.turnGearRatio;
    // turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.MotorOutput.Inverted =
        constants.turnInverted()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));

    // Configure CANCoder
    cancoderConfig.MagnetSensor.MagnetOffset = constants.encoderOffset().getRotations();
    cancoderConfig.MagnetSensor.SensorDirection =
        constants.encoderInverted()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> cancoder.getConfigurator().apply(cancoderConfig, 0.25));

    // Create timestamp queue
    // TODO: mechanical advantage is not using timestamp queue in this place. need to investigate
    // more
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition().clone());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();
    driveTempCelsius = driveTalon.getDeviceTemp();
    driveTempFault = driveTalon.getFault_DeviceTemp();

    // Create turn status signals
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnMagnetHealth = cancoder.getMagnetHealth();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon.getPosition().clone());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnSupplyCurrent = turnTalon.getSupplyCurrent();
    turnTorqueCurrent = turnTalon.getTorqueCurrent();
    turnTempCelsius = turnTalon.getDeviceTemp();
    turnTempFault = turnTalon.getFault_DeviceTemp();
    turnEncoderSyncStickyFault = turnTalon.getStickyFault_FusedSensorOutOfSync();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        DriveConstants.odometryFrequency, drivePosition, turnPosition, turnAbsolutePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveTorqueCurrent,
        turnMagnetHealth,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrent,
        turnTorqueCurrent,
        turnEncoderSyncStickyFault);
    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0, driveTempCelsius, driveTempFault, turnTempCelsius, turnTempFault);
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon, cancoder));
    PhoenixUtil.registerSignals(
        (DriveConstants.canbus.getName() == "rio") ? false : true,
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveTorqueCurrent,
        driveTempCelsius,
        driveTempFault,
        turnAbsolutePosition,
        turnMagnetHealth,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrent,
        turnTorqueCurrent,
        turnTempCelsius,
        turnEncoderSyncStickyFault);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Update drive motor inputs
    inputs.driveConnected =
        BaseStatusSignal.isAllGood(
            drivePosition,
            driveVelocity,
            driveAppliedVolts,
            driveSupplyCurrent,
            driveTorqueCurrent);
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();
    inputs.driveTempCelsius = driveTempCelsius.getValueAsDouble();
    inputs.driveTempFault = driveTempFault.getValue();

    // Update turn motor inputs
    inputs.turnConnected =
        BaseStatusSignal.isAllGood(
            turnPosition, turnVelocity, turnAppliedVolts, turnSupplyCurrent, turnTorqueCurrent);
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
    inputs.turnTorqueCurrentAmps = turnTorqueCurrent.getValueAsDouble();
    inputs.turnTempCelsius = turnTempCelsius.getValueAsDouble();
    inputs.turnTempFault = turnTempFault.getValue();

    // Update turn encoder inputs
    inputs.turnEncoderConnected =
        BaseStatusSignal.isAllGood(turnAbsolutePosition, turnMagnetHealth);
    // TODO: MA subtracts the encoder offset, this would normalize the values. should we do it? Or
    // does the cancoder already do this?
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnMagnetHealth = turnMagnetHealth.getValue();
    inputs.turnEncoderSyncStickyFault = turnEncoderSyncStickyFault.getValue();

    // Update odometry inputs
    // TODO: MA does this different. investigate
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnTalon.setControl(torqueCurrentRequest.withOutput(output));
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    // TODO: how to hadle ff, kS or in this method
    driveTalon.setControl(
        velocityTorqueCurrentRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnTalon.setControl(positionTorqueCurrentRequest.withPosition(rotation.getRotations()));
  }

  @Override
  public void setDrivePID(Slot0Configs config) {
    driveConfig.Slot0 = config;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
  }

  @Override
  public void setTurnPID(Slot0Configs config) {
    turnConfig.Slot0 = config;
    tryUntilOk(5, () -> turnTalon.getConfigurator().apply(turnConfig, 0.25));
  }

  // TODO: do we want a setBrakeMode method?
}
