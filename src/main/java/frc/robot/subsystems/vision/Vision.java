package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.AprilTagLayoutType;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedSupplier;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlert;
  private final Alert poseNotInitializedAlert =
      new Alert(
          "Robot pose not initialized. Face robot toward 2 april tags. If all cameras are disconnected, press the back button with reef camera facing away from driver station. (This can be done while disabled)",
          AlertType.kError);

  public Vision(
      VisionConsumer consumer,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedSupplier,
      VisionIO... io) {
    this.consumer = consumer;
    this.poseSupplier = poseSupplier;
    this.chassisSpeedSupplier = chassisSpeedSupplier;
    this.io = io;

    // Initialize inputs
    // and disconnected alerts
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    this.disconnectedAlert = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
      disconnectedAlert[i] = new Alert(inputs[i].name + " is disconnected.", AlertType.kWarning);
    }

    // Set rotation not initialized alert
    poseNotInitializedAlert.set(Constants.getRobot() == RobotType.SIMBOT ? false : true);
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  public Pose2d getTagPose(int cameraIndex) {
    Pose2d tagPose2d;

    if (inputs[cameraIndex].tagIds.length > 0) {
      int tagId = inputs[cameraIndex].tagIds[0];
      var tagPose = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(tagId);

      if (tagPose.isPresent()) {
        tagPose2d = tagPose.get().toPose2d();
      } else {
        tagPose2d = new Pose2d();
      }
    } else {
      tagPose2d = new Pose2d();
    }
    return tagPose2d;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + inputs[i].name, inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlert[cameraIndex].set(
          !inputs[cameraIndex].connected && Constants.getMode() != Mode.SIM);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPosesMT1 = new LinkedList<>();
      List<Pose3d> robotPosesMT2 = new LinkedList<>();
      List<Pose3d> robotPosesAcceptedMT1 = new LinkedList<>();
      List<Pose3d> robotPosesRejectedMT1 = new LinkedList<>();
      List<Pose3d> robotPosesAcceptedMT2 = new LinkedList<>();
      List<Pose3d> robotPosesRejectedMT2 = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Reject pose and do not log pose if 0 tags are seen
        if (observation.tagCount() == 0) {
          continue;
        }
        // Check whether to reject pose
        boolean rejectPose =
            (observation.type() == PoseObservationType.MEGATAG_1
                    && observation.tagCount() == 1) // MegaTag1 poses must see 2 tags
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate
                || Math.abs(
                        Units.radiansToDegrees(chassisSpeedSupplier.get().omegaRadiansPerSecond))
                    > maxRotVelocity // Must not be rotating too fast

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > FieldConstants.fieldLength
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > FieldConstants.fieldWidth;

        // Add pose to log
        if (observation.type() == PoseObservationType.MEGATAG_1) {
          robotPosesMT1.add(observation.pose());
          if (rejectPose) {
            robotPosesRejectedMT1.add(observation.pose());
          } else {
            robotPosesAcceptedMT1.add(observation.pose());
          }
        } else if (observation.type() == PoseObservationType.MEGATAG_2) {
          robotPosesMT2.add(observation.pose());
          if (rejectPose) {
            robotPosesRejectedMT2.add(observation.pose());
          } else {
            robotPosesAcceptedMT2.add(observation.pose());
          }
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_1) {
          linearStdDev *= linearStdDevMegatag1Factor;
          angularStdDev *= angularStdDevMegatag1Factor;
        } else if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));

        // Pose not initialized alert
        if (poseNotInitializedAlert.get()) {
          poseNotInitializedAlert.set(
              poseSupplier
                          .get()
                          .getTranslation()
                          .minus(observation.pose().getTranslation().toTranslation2d())
                          .getNorm()
                      < translationInitializedError
                  && Math.abs(
                          poseSupplier.get().getRotation().getRadians()
                              - observation.pose().getRotation().getAngle())
                      < rotationInitializedError);
        }
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/" + inputs[cameraIndex].name + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/" + inputs[cameraIndex].name + "/MegaTag1/RobotPoses",
          robotPosesMT1.toArray(new Pose3d[robotPosesMT1.size()]));
      Logger.recordOutput(
          "Vision/" + inputs[cameraIndex].name + "/MegaTag2/RobotPoses",
          robotPosesMT2.toArray(new Pose3d[robotPosesMT2.size()]));
      Logger.recordOutput(
          "Vision/" + inputs[cameraIndex].name + "/MegaTag1/RobotPosesAccepted",
          robotPosesAcceptedMT1.toArray(new Pose3d[robotPosesAcceptedMT1.size()]));
      Logger.recordOutput(
          "Vision/" + inputs[cameraIndex].name + "/MegaTag1/RobotPosesRejected",
          robotPosesRejectedMT1.toArray(new Pose3d[robotPosesRejectedMT1.size()]));
      Logger.recordOutput(
          "Vision/" + inputs[cameraIndex].name + "/MegaTag2/RobotPosesAccepted",
          robotPosesAcceptedMT2.toArray(new Pose3d[robotPosesAcceptedMT2.size()]));
      Logger.recordOutput(
          "Vision/" + inputs[cameraIndex].name + "/MegaTag2/RobotPosesRejected",
          robotPosesRejectedMT2.toArray(new Pose3d[robotPosesRejectedMT2.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPosesMT1);
      allRobotPoses.addAll(robotPosesMT2);
      allRobotPosesAccepted.addAll(robotPosesAcceptedMT1);
      allRobotPosesRejected.addAll(robotPosesRejectedMT1);
      allRobotPosesAccepted.addAll(robotPosesAcceptedMT2);
      allRobotPosesRejected.addAll(robotPosesRejectedMT2);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

    /*

    if ()




    Logger.recordOutput(
        "Vision/Object/ObjectPoses", all object poses);
    Logger.recordOutput(
        "Vision/Object/TargetObjectPose", target object pose);
        */
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
