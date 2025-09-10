package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CanBus;
import lombok.Builder;

public class DriveConstants {
  public static final CanBus canbus;

  static {
    canbus = CanBus.rio;
  }

  public static final double odometryFrequency =
      new CANBus(canbus.getName()).isNetworkFD() ? 250.0 : 100.0;
  public static final double trackWidthX = Units.inchesToMeters(19.5);
  public static final double trackWidthY = Units.inchesToMeters(22.5);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  // TODO: calculate max speed and acceleration
  public static final double maxLinearSpeed = 4.69;
  public static final double maxAngularSpeed = 4.69 / driveBaseRadius;
  public static final double maxLinearAcceleration = 22.0;

  /** Includes bumpers! */
  public static final double robotWidth = Units.inchesToMeters(36);

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  // TODO: calculate this
  public static final double wheelRadius = Units.inchesToMeters(2);

  // TODO: fix this
  //   public static final ModuleLimits moduleLimitsFree =
  //       new ModuleLimits(maxLinearSpeed, maxLinearAcceleration, Units.degreesToRadians(1080.0));

  public static final ModuleConfig[] moduleConfigsComp = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(1)
        .turnMotorId(2)
        .encoderId(9)
        .encoderOffset(Rotation2d.fromDegrees(0.15234375))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(4)
        .encoderId(10)
        .encoderOffset(Rotation2d.fromDegrees(-0.4873046875))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(5)
        .turnMotorId(6)
        .encoderId(11)
        .encoderOffset(Rotation2d.fromDegrees(-0.219482421875))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(7)
        .turnMotorId(8)
        .encoderId(12)
        .encoderOffset(Rotation2d.fromDegrees(0.17236328125))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // Spare
    // This should only be used at a competition if the 5th module is needed
    // To use this module modify the ecoder offest and swap the index used in RobotContainer
    ModuleConfig.builder()
        .driveMotorId(90)
        .turnMotorId(91)
        .encoderId(92)
        // Modify encoder offset using the values below
        // (FL +0) (FR +90) (BL +270) (BR +180)
        // TODO: verify these numbers are correct
        .encoderOffset(Rotation2d.fromDegrees(0.17236328125 + 0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static final ModuleConfig[] moduleConfigsDev = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(1)
        .turnMotorId(2)
        .encoderId(9)
        .encoderOffset(Rotation2d.fromDegrees(0.15234375))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(4)
        .encoderId(10)
        .encoderOffset(Rotation2d.fromDegrees(-0.4873046875))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(5)
        .turnMotorId(6)
        .encoderId(11)
        .encoderOffset(Rotation2d.fromDegrees(-0.219482421875))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(7)
        .turnMotorId(8)
        .encoderId(12)
        .encoderOffset(Rotation2d.fromDegrees(0.17236328125))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static class PigeonConstants {
    public static final int id = 14;
  }

  @Builder
  public record ModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderId,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {}
}
