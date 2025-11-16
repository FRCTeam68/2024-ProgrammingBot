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
import java.util.ArrayList;
import java.util.List;

public class AutonSequenceCenter implements AutonSequence {
  @Override
  public String getAutonChooserName() {
    return "Center";
  }

  private enum Path {
    side("CenterPath");

    String name;

    Path(String name) {
      this.name = name;
    }
  }

  @Override
  public List<String> getPathNames() {
    List<String> pathNames = new ArrayList<>();

    for (Path path : Path.values()) {
      pathNames.add(path.name);
    }

    return pathNames;
  }

  @Override
  public Command sequence(
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
