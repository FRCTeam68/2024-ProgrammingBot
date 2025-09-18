package frc.robot.commands.Devbot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Devbot.wrist.Wrist;
import frc.robot.subsystems.rollers.RollerSystem;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class IntakeCommands {
  public static Command manualIntake(
      RollerSystem intake,
      RollerSystem feederLower,
      RollerSystem feederUpper,
      Wrist wrist,
      BooleanSupplier haveNote) {
    Set<Subsystem> set;
    if (!haveNote.getAsBoolean()) {
      set = Set.of(intake, feederLower, feederUpper, wrist);
    } else {
      set = Set.of();
    }
    return new DeferredCommand(
        () -> {
          Command command;
          if (!haveNote.getAsBoolean()) {
            command =
                Commands.sequence(
                    Commands.runOnce(() -> wrist.setPosition(45)),
                    Commands.waitUntil(() -> wrist.atPosition()),
                    Commands.runOnce(() -> intake.setSpeed(100)),
                    Commands.runOnce(() -> feederLower.setSpeed(100)),
                    Commands.runOnce(() -> feederUpper.setSpeed(50)),
                    // Commands.waitUntil(intake.objectDetected),
                    // Commands.runOnce(() -> set object in intake flag),
                    Commands.waitUntil(() -> haveNote.getAsBoolean()),
                    // Commands.runOnce(() -> set object fully in robot flag),
                    Commands.runOnce(() -> intake.stop()),
                    Commands.runOnce(() -> feederLower.stop()),
                    Commands.runOnce(() -> feederUpper.stop()));
            // .onlyIf(() -> !haveNote.getAsBoolean());
            // command.addRequirements(intake, feederLower, feederUpper, wrist);
          } else {
            command = Commands.none();
          }
          return command;
        },
        set);
  }
}
