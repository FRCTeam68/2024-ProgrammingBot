package frc.robot.subsystems.templates.sensors;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.PhoenixUtil;

public class RangeSensorIOCANrange implements RangeSensorIO {
  private final CANrange canrange;

  private final StatusSignal<Boolean> detected;
  private final StatusSignal<Distance> distance;

  public RangeSensorIOCANrange(Integer id, CANBus canbus, CANrangeConfiguration config) {
    canrange = new CANrange(id, canbus);

    tryUntilOk(5, () -> canrange.getConfigurator().apply(config));

    detected = canrange.getIsDetected();
    distance = canrange.getDistance();
    canrange.getAmbientSignal();
    canrange.getDistanceStdDev();
    canrange.getSignalStrength();

    tryUntilOk(5, () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, detected, distance));
    tryUntilOk(5, () -> ParentDevice.optimizeBusUtilizationForAll(canrange));
    PhoenixUtil.registerSignals((canbus.getName() == "rio") ? false : true, detected, distance);
  }

  @Override
  public void updateInputs(RangeSystemIOInputs inputs) {
    inputs.connected = BaseStatusSignal.isAllGood(detected, distance);
    inputs.detected = detected.getValue();
    inputs.distance_mm = distance.getValueAsDouble();
  }
}
