package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleBinaryOperator;

public class VisionConstants {
  // Camera information
  public static enum CameraInfo {
    // Values:
    // Name - must match name configured on coprocessor
    // April tag pose standard deviation multiplier
    // Camera pose relative to the robot
    // Object detection edges - bottom, top, left, right
    LL_2("limelight-two", 1.0, new Pose3d(), new double[] {}),
    LL_3G("limelight-threeg", 1.0, new Pose3d(), new double[] {}),
    LL_4("limelight-four", 1.0, new Pose3d(), new double[] {0, 100, 0, 100});

    String name;
    double MTStdDevFactor;
    Pose3d pose;
    double[] objectDetectionEdges;

    CameraInfo(String name, double MTStdDevFactor, Pose3d pose, double[] objectDetectionEdges) {
      this.name = name;
      this.MTStdDevFactor = MTStdDevFactor;
      this.pose = pose;
      this.objectDetectionEdges = objectDetectionEdges;
    }
  }

  // Pose filtering thresholds
  public static final double MT1MinTags = 2;
  public static final double MT1MaxLinearVelocity = 0.5; // Meters per second
  public static final double MT1MaxAngularVelocity =
      Units.degreesToRadians(5); // Radians per second
  public static final double MT1MaxAverageTagDistance = 2.0; // Meters
  public static final double maxZError = 0.5; // Meters

  // Higher standard deviations result in less truested data

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static final double linearStdDevBaseline = 0.02; // Meters
  public static final double angularStdDevBaseline = 0.06; // Radians

  // Multipliers to apply for MegaTag 1 observations
  public static final double linearStdDevMegatag1Factor =
      Double.POSITIVE_INFINITY; // Do not use linear data
  public static final double angularStdDevMegatag1Factor = 1.0;

  // Multipliers to apply for MegaTag 2 observations
  public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static final double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  // Object Detection
  // TODO: do we want to have a confidence threshold here. It is already in the ll setup. It could
  // be confusing.
  // List of object tags in the same order of the file uploaded to the limelight
  public static enum ObjectObservationType {
    ALGAE,
    CORAL
  }

  public static final DoubleBinaryOperator algaeDistanceEquation =
      (x, y) -> Math.pow(Math.min(x, y), 2) + 10;
  public static final DoubleBinaryOperator coralDistanceEquation =
      (x, y) -> Math.pow(Math.min(x, y), 2) + 10;

  // Pipelines - same for every limelight
  public static enum PipelineType {
    MEGATAG_2(0),
    MEGATAG_1_2(1),
    OBJECT(2);

    int index;

    PipelineType(int index) {
      this.index = index;
    }
  }
}
