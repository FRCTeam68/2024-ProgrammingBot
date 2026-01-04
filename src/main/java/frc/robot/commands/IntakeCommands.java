package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotState;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.sensors.NoteSensor;
import frc.robot.subsystems.wrist.Wrist;
import java.util.Set;

public class IntakeCommands {
  /** Intake note from the ground. This will only end when a note is seen by the sensor. */
  public static Command intake(
      Wrist wrist,
      RollerSystem intake,
      RollerSystem feederLower,
      RollerSystem feederUpper,
      NoteSensor noteSensor) {
    // Configure subsystem requirements
    Set<Subsystem> requirements;
    if (RobotState.haveNote || noteSensor.isDetected()) {
      requirements = Set.of();
    } else {
      requirements = Set.of(wrist, intake, feederLower, feederUpper);
    }

    return Commands.defer(
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
                        Commands.runOnce(() -> wrist.runPosition(Wrist.getIntake().get())),
                        Commands.waitUntil(() -> wrist.atSetpoint()),
                        Commands.runOnce(() -> intake.runVolts(7)),
                        Commands.runOnce(() -> feederLower.runVolts(7)),
                        Commands.runOnce(() -> feederUpper.runVolts(0.6)),
                        Commands.waitUntil(() -> noteSensor.isDetected()),
                        Commands.runOnce(() -> RobotState.haveNote = true))
                    .finallyDo(
                        () -> {
                          intake.stop();
                          feederLower.stop();
                          feederUpper.stop();
                        });
          }

          // execute command
          return command;
        },
        requirements);
  }

  /** Set haveNote to false and eject any note out the intake. */
  public static Command outtake(
      Wrist wrist, RollerSystem intake, RollerSystem feederLower, RollerSystem feederUpper) {
    return Commands.sequence(
            Commands.runOnce(() -> RobotState.haveNote = false),
            Commands.runOnce(() -> wrist.runPosition(Wrist.getOuttake().get())),
            Commands.waitUntil(() -> wrist.atSetpoint()),
            Commands.runOnce(
                () -> {
                  intake.runVolts(-7);
                  feederLower.runVolts(-7);
                  feederUpper.runVolts(-7);
                }),
            Commands.idle())
        .finallyDo(
            () -> {
              intake.stop();
              feederLower.stop();
              feederUpper.stop();
            });
  }
}
