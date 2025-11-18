package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.sensors.NoteSensor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import java.util.List;

public interface AutonSequence {
  default List<String> getPathNames() {
    return null;
  }

  default Command sequence(
      Drive drive,
      Wrist wrist,
      Shooter shooter,
      RollerSystem intake,
      RollerSystem feederLower,
      RollerSystem feederUpper,
      NoteSensor noteSensor) {
    return null;
  }
}
