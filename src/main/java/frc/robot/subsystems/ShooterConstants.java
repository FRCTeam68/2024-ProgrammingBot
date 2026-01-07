package frc.robot.subsystems;

import frc.robot.Constants;
import java.util.function.DoubleUnaryOperator;

public final class ShooterConstants {
  public static final ShooterConfig amp;
  public static final ShooterConfig subwoofer;
  public static final ShooterConfig stage;
  public static final ShooterConfig trap;

  public static final ShooterConfig[] dynamicSpeakerSamples;
  public static final DoubleUnaryOperator dynamicElevationEquation;
  public static final DoubleUnaryOperator dynamicUpperVelocityEquation;
  public static final DoubleUnaryOperator dynamicLowerVelocitynEquation;

  static {
    switch (Constants.fieldtype) {
      case OFFICIAL -> {
        amp = new ShooterConfig(0, 0, 0);
        subwoofer = new ShooterConfig(0, 0, 0);
        stage = new ShooterConfig(0, 0, 0);
        trap = new ShooterConfig(0, 0, 0);

        dynamicSpeakerSamples = new ShooterConfig[] {};
        dynamicElevationEquation = (x) -> Math.pow(x, 2) + 10;
        dynamicUpperVelocityEquation = (x) -> Math.pow(x, 2) + 10;
        dynamicLowerVelocitynEquation = (x) -> Math.pow(x, 2) + 10;
      }
      default -> {
        amp = new ShooterConfig(25, 10, 10);
        subwoofer = new ShooterConfig(0, 0, 0);
        stage = new ShooterConfig(0, 0, 0);
        trap = new ShooterConfig(0, 0, 0);

        dynamicSpeakerSamples = new ShooterConfig[] {};
        dynamicElevationEquation = (x) -> Math.pow(x, 2) + 10;
        dynamicUpperVelocityEquation = (x) -> Math.pow(x, 2) + 10;
        dynamicLowerVelocitynEquation = (x) -> Math.pow(x, 2) + 10;
      }
    }
  }

  public static class ShooterConfig {
    public double elevation;
    public double upperVelocity;
    public double lowerVelocity;

    public ShooterConfig(double elevation, double upperVelocity, double lowerVelocity) {
      this.elevation = elevation;
      this.upperVelocity = upperVelocity;
      this.lowerVelocity = lowerVelocity;
    }
  }
}
