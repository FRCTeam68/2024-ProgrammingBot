package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.wrist.Wrist;
import java.util.Set;

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

  public static Command runWrist(Wrist system, double offset) {
    return Commands.runOnce(() -> system.runPosition(offset));
  }

  public static Command subsystem1(
      RollerSystem intake, RollerSystem feederLower, RollerSystem feederUpper) {
    return Commands.sequence(
            Commands.runOnce(() -> feederUpper.runVolts(2), feederUpper),
            Commands.waitUntil(() -> 1 == 2))
        .beforeStarting(() -> System.out.println("1 - started"))
        .finallyDo(() -> System.out.println("1 - ended"));
  }

  public static Command subsystem2(
      RollerSystem intake, RollerSystem feederLower, RollerSystem feederUpper) {
    Command command =
        Commands.sequence(
                Commands.runOnce(() -> feederLower.runVolts(2)), Commands.waitUntil(() -> 1 == 2))
            .beforeStarting(() -> System.out.println("2 - started"))
            .finallyDo(() -> System.out.println("2 - ended"));
    command.addRequirements(feederLower);
    return command;
  }

  public static Command subsystem3(
      RollerSystem intake, RollerSystem feederLower, RollerSystem feederUpper) {
    return Commands.defer(
        () -> {
          Command command =
              Commands.sequence(
                      Commands.runOnce(() -> intake.runVolts(2)), Commands.waitUntil(() -> 1 == 2))
                  .beforeStarting(() -> System.out.println("3 - started"))
                  .finallyDo(() -> System.out.println("3 - ended"));
          return command;
        },
        Set.of(intake));
  }

  public static Command subsystem4(
      RollerSystem intake, RollerSystem feederLower, RollerSystem feederUpper) {
    return Commands.runOnce(() -> {}, intake, feederLower, feederUpper)
        .beforeStarting(() -> System.out.println("4 - started"))
        .finallyDo(() -> System.out.println("4 - ended"));
  }
}
