package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil.ControlMode;

public class WristIOSim implements WristIO {
  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);

  private SlotConfigs[] slotConfigs = new SlotConfigs[3];
  private ControlMode mode = ControlMode.Neutral;
  private double appliedVoltage = 0.0;

  public WristIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getFalcon500(2), .1, WristIOTalonFX.getReduction()),
            DCMotor.getFalcon500(2));
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setVolts(0.0);
    } else {
      if (mode == ControlMode.Position) {
        setInputVoltage(controller.calculate(sim.getAngularPositionRotations()));
      }
    }

    sim.update(Constants.loopPeriodSecs);

    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.elevationDeg = Units.rotationsToDegrees(sim.getAngularPositionRotations());
    inputs.velocityDegPerSec = Units.rotationsToDegrees(sim.getAngularVelocityRPM() / 60.0);
    inputs.leaderAppliedVoltage = appliedVoltage;
    // TODO: does this need to be divided by number of motors?
    inputs.leaderSupplyCurrentAmps = sim.getCurrentDrawAmps() / 2.0;
    inputs.leaderTorqueCurrentAmps = sim.getCurrentDrawAmps() * 12.0 / appliedVoltage / 2.0;
    inputs.followerAppliedVoltage = appliedVoltage;
    inputs.followerSupplyCurrentAmps = sim.getCurrentDrawAmps() / 2.0;
    inputs.followerTorqueCurrentAmps = sim.getCurrentDrawAmps() * 12.0 / appliedVoltage / 2.0;
  }

  @Override
  public void setVolts(double volts) {
    mode = ControlMode.Voltage;
    setInputVoltage(volts);
  }

  @Override
  public void setPosition(double position, int slot) {
    mode = ControlMode.Position;
    controller.setPID(slotConfigs[slot].kP, slotConfigs[slot].kI, slotConfigs[slot].kD);
    controller.setSetpoint(Units.degreesToRotations(position));
  }

  @Override
  public void stop() {
    setVolts(0.0);
  }

  @Override
  public void zero(double offset) {
    controller.setSetpoint(offset);
    sim.setAngle(Units.degreesToRadians(offset));
  }

  @Override
  public void setPID(SlotConfigs... newConfig) {
    for (int i = 0; i < newConfig.length; i++) {
      slotConfigs[i] = newConfig[i];
    }
  }

  private void setInputVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }
}
