package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.sensors.NoteSensor;
import frc.robot.subsystems.wrist.Wrist;

public class IntakeCommands {
  public static Command intake(
      Wrist wrist,
      RollerSystem intake,
      RollerSystem feederLower,
      RollerSystem feederUpper,
      NoteSensor noteSensor) {
    // If already have note is true, do nothing.
    if (RobotState.haveNote) {
      return Commands.none();
    }

    // If note sensor is detected, set haveNote to true and do nothing else.
    if (noteSensor.isDetected()) {
      RobotState.haveNote = true;
      return Commands.none();
    }

    // Start rollers and wait until note is detected.
    Command command =
        Commands.sequence(
            Commands.runOnce(() -> wrist.setPosition(wrist.getIntake().getAsDouble())),
            Commands.waitUntil(() -> wrist.atSetpoint()),
            Commands.runOnce(() -> intake.runVolts(12)),
            Commands.runOnce(() -> feederLower.runVolts(12)),
            Commands.runOnce(() -> feederUpper.runVolts(12)),
            Commands.waitUntil(() -> noteSensor.isDetected()),
            Commands.runOnce(() -> intake.stop()),
            Commands.runOnce(() -> feederLower.stop()),
            Commands.runOnce(() -> feederUpper.stop()),
            Commands.runOnce(() -> RobotState.haveNote = true));
    command.addRequirements(wrist, intake, feederLower, feederUpper);
    return command;
  }
}
