package frc.robot.util;

import java.util.List;

public class AutonConfig {
  public AutonSequence sequence;
  public List<String> pathNames;

  public AutonConfig(AutonSequence sequence, String... pathNames) {
    this.sequence = sequence;
    this.pathNames = List.of(pathNames);
  }

  public static enum AutonSequence {
    Center
  }
}
