package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.FieldConstants.FieldType;

public final class Constants {
  private static final Mode simType = Mode.SIM;
  public static final FieldType fieldtype = FieldType.PRACTICE;
  public static final Boolean tuningMode = true;
  public static final Double loopPeriodSecs = 0.02;
  public static final Double warningTempCelsius = 60.0;

  public static Mode getMode() {
    if (RobotBase.isReal()) {
      return Mode.REAL;
    } else {
      return simType;
    }
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  /** Checks that the default robot is selected and tuning mode is disabled. */
  public static class CheckPullRequest {
    public static void main(String... args) {
      if (tuningMode) {
        System.err.println("Do not merge, non-default constants are configured.");
        System.exit(1);
      }
    }
  }
}
