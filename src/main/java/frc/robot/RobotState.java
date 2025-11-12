package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {
  // TODO: should this use getter and setter or be public
  @AutoLogOutput(key = "RobotState/HaveNote")
  public static boolean haveNote = false;
}
