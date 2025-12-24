package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.Wrist;

public class ManipulatorCommands {
  public static Command wristZeroByAmps(Wrist wrist) {
    Command command =
        Commands.sequence(
            Commands.runOnce(() -> wrist.runVolts(3)),
            Commands.waitUntil(() -> wrist.getLeaderTorqueCurrent() > 5),
            Commands.runOnce(() -> wrist.stop()),
            Commands.runOnce(() -> wrist.setPosition(Wrist.getMaximum())));
    command.addRequirements(wrist);
    return command;
  }
}
