package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.Wrist;

public class ManipulatorCommands {
  public static Command wristZeroByAmps(Wrist wrist) {
    Command command =
        Commands.sequence(
            // TODO: should we add the subsystem to the runonce commands
            Commands.runOnce(() -> wrist.setVolts(3)),
            Commands.waitUntil(() -> wrist.getLeaderTorqueCurrent() > 20),
            Commands.runOnce(() -> wrist.stop()),
            Commands.runOnce(() -> wrist.zero(wrist.getMaximum()))
            // TODO: does this work
            // Commands.runOnce(() -> {
            //     wrist.stop();
            //     wrist.zero(wrist.getMaximum());
            // })
            );
    // TODO: will it automatically add the requirments
    command.addRequirements(wrist);
    return command;
  }
}
