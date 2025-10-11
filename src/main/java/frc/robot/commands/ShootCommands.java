package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterConstants.ShooterConfig;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.wrist.Wrist;

public class ShootCommands {
  public static Command defaultShooterVelocity(Shooter shooter) {
    return Commands.runOnce(() -> shooter.setVelocity(10, 10));
  }

  /**
   * Manually run upper feeder to shoot note. This can cause a jam if the shooter is not running.
   * Turns off upper feeder when ended.
   */
  public static Command shootManual(RollerSystem feederUpper) {
    return Commands.run(() -> feederUpper.setVolts(12), feederUpper)
        .finallyDo(() -> feederUpper.setVolts(0));
  }

  public static Command shootAutomatic(Shooter shooter, RollerSystem feederUpper) {
    return Commands.sequence(
        Commands.waitUntil(() -> shooter.atSetpoint()),
        Commands.runOnce(() -> feederUpper.setVolts(12)),
        Commands.waitSeconds(2),
        Commands.runOnce(() -> feederUpper.setVolts(12)));
  }

  public static Command setStaticShotConfig(Shooter shooter, Wrist wrist, ShooterConfig config) {
    return Commands.runOnce(
        () -> {
          wrist.setPosition(config.elevation);
          shooter.setVelocity(config.upperVelocity, config.lowerVelocity);
        },
        shooter,
        wrist);
  }
}
