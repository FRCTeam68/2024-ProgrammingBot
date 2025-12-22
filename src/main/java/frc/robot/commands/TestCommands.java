package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;

public class TestCommands {
  public static Command runRoller(RollerSystem system) {
    return Commands.sequence(
        Commands.runOnce(() -> system.runVolts(2)),
        Commands.waitSeconds(2),
        Commands.runOnce(() -> system.runVolts(0)));
  }

  public static Command runShooter(Shooter system) {
    return Commands.sequence(
        Commands.runOnce(() -> system.runVolts(2, 2)),
        Commands.waitSeconds(2),
        Commands.runOnce(() -> system.runVolts(0, 0)));
  }
}
