package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import org.littletonrobotics.junction.Logger;

public class CANUtil {
  private static CANBus rioBus = new CANBus("rio");
  private static CANBus canivoreBus;

  public static void registerCanivore() {
    canivoreBus = new CANBus("*");
  }

  public static void logStatus() {
    CANBusStatus rioInfo = rioBus.getStatus();
    Logger.recordOutput("CANBus/Rio/Util", rioInfo.BusUtilization);
    Logger.recordOutput("CANBus/Rio/Status", rioInfo.Status.getName());
    if (!rioInfo.Status.isOK())
      Logger.recordOutput("CANBus/Rio/Desc", rioInfo.Status.getDescription());

    if (canivoreBus != null) {
      CANBusStatus canivoreInfo = canivoreBus.getStatus();
      Logger.recordOutput("CANBus/Canivore/Util", canivoreInfo.BusUtilization);
      Logger.recordOutput("CANBus/Canivore/Status", canivoreInfo.Status.getName());
      if (!canivoreInfo.Status.isOK())
        Logger.recordOutput("CANBus/Canivore/Desc", canivoreInfo.Status.getDescription());
    }
  }
}
