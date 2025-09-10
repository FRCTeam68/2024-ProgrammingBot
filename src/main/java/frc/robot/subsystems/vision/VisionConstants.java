package frc.robot.subsystems.vision;

// High standard deviations result in less truested data
public class VisionConstants {
  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "limelight-donotuse"; // back,  LL2
  public static String camera1Name = "limelight-reef"; // front, LL3G

  public static double translationInitializedError = 0.2; // Meters
  public static double rotationInitializedError = 2.5; // Degrees

  // Basic filtering thresholds
  public static double maxZError = 0.75; // Meters
  public static double maxRotVelocity = 250; // Degrees per second

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        0.5, // Camera 0  LL2
        1.0 // Camera 1   LL3G
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag1Factor =
      Double.POSITIVE_INFINITY; // Do not use linear data
  public static double angularStdDevMegatag1Factor = 1.0;

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
