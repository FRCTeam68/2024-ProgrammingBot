package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.sensors.NoteSensor;
import frc.robot.subsystems.wrist.Wrist;
import java.util.Set;

public class IntakeCommands {
  public static Command intake(
      Wrist wrist,
      RollerSystem intake,
      RollerSystem feederLower,
      RollerSystem feederUpper,
      NoteSensor noteSensor) {
    return new DeferredCommand(
        () -> {
          // initialization
          Command command = Commands.none();

          // if note is in robot move nothing and set haveNote to true
          if (!RobotState.haveNote && noteSensor.isDetected()) {
            RobotState.haveNote = true;
          }

          // intake note
          if (!RobotState.haveNote) {
            command =
                Commands.sequence(
                    Commands.runOnce(() -> wrist.runPosition(wrist.getIntake().getAsDouble())),
                    Commands.waitUntil(() -> wrist.atSetpoint()),
                    Commands.runOnce(() -> intake.runVolts(7)),
                    Commands.runOnce(() -> feederLower.runVolts(7)),
                    Commands.runOnce(() -> feederUpper.runVolts(1)),
                    Commands.waitUntil(() -> noteSensor.isDetected()),
                    Commands.runOnce(() -> intake.stop()),
                    Commands.runOnce(() -> feederLower.stop()),
                    Commands.runOnce(() -> feederUpper.stop()),
                    Commands.runOnce(() -> RobotState.haveNote = true));
          }

          // execute command
          return command;
        },
        Set.of(wrist, intake, feederLower, feederUpper));
  }

  public static Command outtake(
      Wrist wrist, RollerSystem intake, RollerSystem feederLower, RollerSystem feederUpper) {
    return Commands.sequence(
            Commands.runOnce(() -> RobotState.haveNote = false),
            Commands.runOnce(() -> wrist.runPosition(wrist.getIntake().get())),
            Commands.waitUntil(() -> wrist.atSetpoint()),
            Commands.runOnce(
                () -> {
                  intake.runVolts(-7);
                  feederLower.runVolts(-7);
                  feederUpper.runVolts(-7);
                }))
        .finallyDo(
            () ->
                Commands.runOnce(
                    () -> {
                      intake.runVolts(0);
                      feederLower.runVolts(0);
                      feederUpper.runVolts(0);
                    }));
  }
}
