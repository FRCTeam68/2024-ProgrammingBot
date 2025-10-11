package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public CameraType cameraType = null;
    public String name = "";
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
    public ObjectObservation[] objectObservations = new ObjectObservation[0];
  }

  public static enum CameraType {
    LL_2,
    LL_3G,
    LL_4
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  // TODO: are we every going to use this. we would just use poses instead
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
  }

  /** Represents an object sample. */
  public static record ObjectObservation(
      double txCenter, double tyCenter, double width, double height, double confidence) {}

  public static enum ObjectObservationType {
    Coral,
    Algae,
    Other
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
