package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.wrist.Wrist;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class IntakeCommands {
  public static Command intake(
      RollerSystem intake,
      RollerSystem feederLower,
      RollerSystem feederUpper,
      Wrist wrist,
      BooleanSupplier haveNote) {
    Set<Subsystem> subsystems;
    if (!haveNote.getAsBoolean()) {
      subsystems = Set.of(intake, feederLower, feederUpper, wrist);
    } else {
      subsystems = Set.of();
    }
    return new DeferredCommand(
        () -> {
          Command command;
          if (!haveNote.getAsBoolean()) {
            command =
                Commands.sequence(
                    Commands.runOnce(() -> wrist.setPosition(wrist.getIntake().getAsDouble())),
                    Commands.waitUntil(() -> wrist.atSetpoint()),
                    Commands.runOnce(() -> intake.setVelocity(100)),
                    Commands.runOnce(() -> feederLower.setVelocity(100)),
                    Commands.runOnce(() -> feederUpper.setVelocity(50)),
                    // Commands.waitUntil(intake.objectDetected),
                    // Commands.runOnce(() -> set object in intake flag),
                    Commands.waitUntil(() -> haveNote.getAsBoolean()),
                    // Commands.runOnce(() -> set object fully in robot flag),
                    Commands.runOnce(() -> intake.stop()),
                    Commands.runOnce(() -> feederLower.stop()),
                    Commands.runOnce(() -> feederUpper.stop()));
          } else {
            command = Commands.none();
          }
          return command;
        },
        subsystems);
  }
}
