package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil.ControlMode;

public class RollerSystemIOSim implements RollerSystemIO {
  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private PID[] PIDValues = new PID[3];
  private ControlMode mode = ControlMode.Neutral;

  private double appliedVoltage = 0.0;

  public RollerSystemIOSim(DCMotor motor, double reduction, double moi) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moi, reduction), motor);
  }

  @Override
  public void updateInputs(RollerSystemIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      setVolts(0.0);
    } else {
      if (mode == ControlMode.Speed) {
        setInputVoltage(controller.calculate(sim.getAngularVelocityRPM() / 60));
      } else if (mode == ControlMode.Position) {
        setInputVoltage(controller.calculate(sim.getAngularPositionRotations()));
      }
    }

    sim.update(Constants.loopPeriodSecs);

    inputs.connected = true;
    inputs.positionRotations = sim.getAngularPositionRotations();
    inputs.velocityRotsPerSec = sim.getAngularVelocityRPM() / 60;
    inputs.appliedVoltage = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVolts(double volts) {
    mode = ControlMode.Voltage;
    setInputVoltage(volts);
  }

  @Override
  public void setSpeed(double speed, int slot) {
    mode = ControlMode.Speed;
    controller.setPID(PIDValues[slot].kP, PIDValues[slot].kI, PIDValues[slot].kD);
    controller.setSetpoint(speed);
  }

  @Override
  public void setPosition(double position, int slot) {
    mode = ControlMode.Position;
    controller.setPID(PIDValues[slot].kP, PIDValues[slot].kI, PIDValues[slot].kD);
    controller.setSetpoint(position);
  }

  @Override
  public void stop() {
    mode = ControlMode.Neutral;
    setVolts(0.0);
  }

  @Override
  public void zero() {
    sim.setAngle(0);
  }

  @Override
  public void setPID(SlotConfigs... newconfig) {
    for (int i = 0; i < Math.min(newconfig.length, 3); i++) {
      PIDValues[i] = new PID(newconfig[i].kP, newconfig[i].kI, newconfig[i].kD);
    }
  }

  private void setInputVoltage(double volts) {
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltage);
  }

  // TODO: is there a better central location to put this
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
