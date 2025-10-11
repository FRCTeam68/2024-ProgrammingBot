package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer extends SubsystemBase {
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<Double> wristElevationDeg;
  private final Supplier<Boolean> haveNoteSupplier;

  public NoteVisualizer(
      Supplier<Pose2d> poseSupplier,
      Supplier<Double> wristElevationDeg,
      Supplier<Boolean> haveNoteSupplier) {
    this.poseSupplier = poseSupplier;
    this.wristElevationDeg = wristElevationDeg;
    this.haveNoteSupplier = haveNoteSupplier;
  }

  @Override
  public void periodic() {
    List<Pose3d> notePoses = new LinkedList<>();

    if (haveNoteSupplier.get()) {
      notePoses.add(
          new Pose3d(
                  poseSupplier.get().getX() + 0.0192236344,
                  poseSupplier.get().getY(),
                  0.3160213644,
                  new Rotation3d())
              .rotateAround(
                  new Translation3d(poseSupplier.get().getTranslation())
                      .plus(new Translation3d(-0.0153715466, 0.0, 0.2346029852)),
                  new Rotation3d(0, Units.degreesToRadians(-wristElevationDeg.get()), 0))
              .rotateAround(
                  new Translation3d(poseSupplier.get().getTranslation()),
                  new Rotation3d(poseSupplier.get().getRotation())));
    }

    Logger.recordOutput("RobotPose/Note", notePoses.toArray(new Pose3d[notePoses.size()]));
  }
}
