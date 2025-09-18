package frc.robot.subsystems.Devbot.wrist;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class WristIOSim implements WristIO {
  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private PID[] PIDValues = new PID[3];
  private boolean closedLoop;

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
      if (closedLoop) {
        setInputVoltage(controller.calculate(sim.getAngularPositionRotations()));
      }
    }

    sim.update(Constants.loopPeriodSecs);
    inputs.leaderConnected = true;
    inputs.followerConnected = true;
    inputs.elevationDeg = sim.getAngularPositionRotations();
    inputs.velocityDegPerSec = sim.getAngularVelocityRPM() * 60;
    inputs.leaderAppliedVoltage = appliedVoltage;
    // TODO: does this need to be divided by number of motors?
    inputs.leaderSupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.followerAppliedVoltage = appliedVoltage;
    inputs.followerSupplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVolts(double volts) {
    closedLoop = false;
    setInputVoltage(volts);
  }

  @Override
  public void setPosition(double position, int slot) {
    closedLoop = true;
    controller.setPID(PIDValues[slot].kP, PIDValues[slot].kI, PIDValues[slot].kD);
    controller.setSetpoint(position);
  }

  @Override
  public void stop() {
    setVolts(0.0);
  }

  @Override
  public void zero(double offset) {
    controller.setSetpoint(offset);
    sim.setAngle(Units.rotationsToRadians(offset));
  }

  @Override
  public void setPID(SlotConfigs... newConfig) {
    for (int i = 0; i < newConfig.length; i++) {
      PIDValues[i] = new PID(newConfig[i].kP, newConfig[i].kI, newConfig[i].kD);
    }
  }

  private void setInputVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  private class PID {
    double kP;
    double kI;
    double kD;

    private PID(double kP, double kI, double kD) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
    }
  }
}
