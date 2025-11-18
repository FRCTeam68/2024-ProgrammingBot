package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.sensors.NoteSensor;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AutonUtil;

public class AutonCommands {
  /**
   * Creates a an auton command with the supplied sequence.
   * <li>If in simulation, the robot pose is set to the inital pose of the first path.
   *
   * @return Auton command
   *     <li>If config or config.sequence is null, this will return null.
   */
  public static Command autonCommand(
      Drive drive,
      Wrist wrist,
      Shooter shooter,
      RollerSystem intake,
      RollerSystem feederLower,
      RollerSystem feederUpper,
      NoteSensor noteSensor,
      AutonSequence root) {
    if (Constants.getMode() == Mode.SIM) {
      drive.setPose(AutonUtil.getStartingPose());
      wrist.setPosition(wrist.getStartingElevation());
    }

    RobotState.haveNote = true;

    if (root == null) root = new AutonSequence() {};
    return root.sequence(drive, wrist, shooter, intake, feederLower, feederUpper, noteSensor);
  }

  /**
   * A deadline command that will follow a path and intake. When the path ends, it will delay for
   * the specified time before ending. If a note is intaked, the delay will be canceled and the
   * command will end.
   *
   * @param path
   * @param timeout
   * @return
   */
  public static Command intake(
      RollerSystem intake,
      RollerSystem feederUpper,
      RollerSystem feederLower,
      Wrist wrist,
      NoteSensor noteSensor,
      String pathName,
      double timeout) {
    return Commands.deadline(
        AutonUtil.followPath(pathName)
            .andThen(Commands.waitUntil(() -> RobotState.haveNote).withTimeout(timeout)),
        IntakeCommands.intake(wrist, intake, feederLower, feederUpper, noteSensor));
  }
}
