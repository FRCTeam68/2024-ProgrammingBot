package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootCommands;
import frc.robot.subsystems.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.sensors.NoteSensor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;

public class Center {
  /*
   * Paths:
   * 0: Start -> Close Right
   * 1: Close Right -> Close Middle
   * 2: Close Middle -> Close Left
   */

  public static Command sequence(
      Drive drive,
      Wrist wrist,
      Shooter shooter,
      RollerSystem intake,
      RollerSystem feederLower,
      RollerSystem feederUpper,
      NoteSensor noteSensor) {
    return Commands.sequence(
        // Shoot preload note
        ShootCommands.setStaticShotConfig(shooter, wrist, ShooterConstants.subwoofer),
        Commands.waitUntil(() -> shooter.atSetpoint()),
        Commands.runOnce(() -> feederUpper.setVolts(12)),
        Commands.waitSeconds(2),
        // Collect and shoot close right note
        AutonCommands.intake(intake, feederUpper, feederLower, wrist, noteSensor, 0, 2));
  }
}
