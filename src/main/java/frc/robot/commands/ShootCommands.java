package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.ShooterConstants.ShooterConfig;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.sensors.NoteSensor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class ShootCommands {
  public static Command defaultShooterVelocity(Shooter shooter) {
    return Commands.runOnce(() -> shooter.runVelocity(10, 10));
  }

  /**
   * Manually run upper feeder to shoot note. This can cause a jam if the shooter is not running.
   * Turns off upper feeder when ended.
   */
  public static Command shootManual(RollerSystem feederUpper, NoteSensor noteSensor) {
    return Commands.run(() -> feederUpper.runVolts(12), feederUpper)
        .beforeStarting(
            () -> {
              RobotState.haveNote = false;
              noteSensor.setTripped(false);
              noteSensor.setDetected(false);
            })
        .finallyDo(() -> feederUpper.stop());
  }

  public static Command shootAutomatic(
      Shooter shooter, RollerSystem feederUpper, NoteSensor noteSensor) {
    return Commands.sequence(
            Commands.waitUntil(() -> shooter.atSetpoint()),
            Commands.runOnce(() -> feederUpper.runVolts(12)),
            Commands.waitSeconds(2))
        .beforeStarting(
            () -> {
              RobotState.haveNote = false;
              noteSensor.setTripped(false);
              noteSensor.setDetected(false);
            })
        .finallyDo(
            () -> {
              feederUpper.stop();
              shooter.stop();
            });
  }

  public static Command setStaticShotConfig(Shooter shooter, Wrist wrist, ShooterConfig config) {
    return Commands.runOnce(
        () -> {
          wrist.runPosition(config.elevation);
          shooter.runVelocity(config.upperVelocity, config.lowerVelocity);
        },
        shooter,
        wrist);
  }
}
