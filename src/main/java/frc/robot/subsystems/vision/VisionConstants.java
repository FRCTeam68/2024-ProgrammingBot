package frc.robot.subsystems.vision;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleUnaryOperator;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // Camera names, must match names configured on coprocessor
  public static String LL2Name = "limelight-two";
  public static String LL3GName = "limelight-threeg";
  public static String LL4Name = "limelight-four";

  // Pose filtering thresholds
  public static double MT1MinTags = 2;
  public static double MT1MaxLinearVelocity = 0.5; // Meters per second
  public static double MT1MaxAngularVelocity = Units.degreesToRadians(5); // Radians per second
  public static double MT1MaxAverageTagDistance = 2.0; // Meters
  public static double maxZError = 0.5; // Meters

  // Higher standard deviations result in less truested data

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Multipliers to apply for each camera type
  public static double LL2StdDevFactor = 1.0;
  public static double LL3GStdDevFactor = 1.0;
  public static double LL4StdDevFactor = 1.0;

  // Multipliers to apply for MegaTag 1 observations
  public static double linearStdDevMegatag1Factor =
      Double.POSITIVE_INFINITY; // Do not use linear data
  public static double angularStdDevMegatag1Factor = 1.0;

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

    // Object Detection
    // TODO: do we want to have this. It is already in the ll setup. It could be confusing.
    public static double ObjectConfidenceMin = 0;
    public static DoubleBinaryOperator coralDistanceEquation = (x, y) -> Math.pow(Math.min(x, y), 2) + 10;
    public static DoubleBinaryOperator algaeDistanceEquation = (x, y) -> Math.pow(Math.min(x, y), 2) + 10;
    // public static DoubleUnaryOperator objectDistanceEquation = (x) -> Math.pow(x, 2) + 10;
}
