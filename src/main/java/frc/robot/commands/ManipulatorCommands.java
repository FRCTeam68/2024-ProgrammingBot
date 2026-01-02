package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.Wrist;

public class ManipulatorCommands {
  public static Command wristZeroByAmps(Wrist wrist) {
    double rampTime = 1.0;
    double maxVolts = 0.5;
    double currentLimit = 4.0;
    Timer timer = new Timer();

    return Commands.sequence(
            // Disable software limits
            Commands.runOnce(() -> wrist.setSoftwareLimitsEnabled(false)),
            Commands.runOnce(() -> timer.restart()),

            // Run wrist until current spikes
            Commands.run(
                    () -> {
                      double voltage =
                          (timer.get() < rampTime)
                              ? Math.pow(timer.get() / rampTime, 2) * maxVolts
                              : maxVolts;
                      wrist.runVolts(voltage);
                    },
                    wrist)
                .until(() -> wrist.getLeaderTorqueCurrent() > currentLimit),

            // Set the wrist position
            Commands.runOnce(() -> wrist.setPosition(Wrist.getMaximum())))

        // When complete or cancelled, stop and enable software limits
        .finallyDo(
            () -> {
              wrist.stop();
              wrist.setSoftwareLimitsEnabled(true);
            });
  }
}
