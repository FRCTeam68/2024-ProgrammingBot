package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import lombok.Builder;

public class DriveConstants {
  public static final String canbus;
  public static final double trackWidthX; // meters
  public static final double trackWidthY; // meters
  public static final double maxLinearSpeed; // meters/second
  public static final double maxLinearAcceleration; // meters/second^2
  public static final double wheelRadius; // meters
  private static final double mass; // kilograms
  private static final double moi; // kilograms*meters^2

  static {
    switch (Constants.getRobot()) {
      case DEVBOT -> {
        canbus = "rio";
        trackWidthX = Units.inchesToMeters(19.5);
        trackWidthY = Units.inchesToMeters(22.5);
        maxLinearSpeed = 4.0;
        maxLinearAcceleration = 22.0;
        wheelRadius = Units.inchesToMeters(2);
        mass = 68;
        moi = 1;
      }
      default -> {
        canbus = "*";
        trackWidthX = Units.inchesToMeters(19.5);
        trackWidthY = Units.inchesToMeters(22.5);
        maxLinearSpeed = 4.0;
        maxLinearAcceleration = 22.0;
        wheelRadius = Units.inchesToMeters(2);
        mass = 68;
        moi = 1;
      }
    }
  }

  public static final double odometryFrequency = new CANBus(canbus).isNetworkFD() ? 250.0 : 100.0;
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxAngularSpeed = maxLinearSpeed / driveBaseRadius;

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  // Simple profiled PID configuration
  public static final PIDConstants pidConstantsFast = new PIDConstants(5, 0.0, 0.0);

  // Autopilot configuration
  public static final APProfile autoPilot =
      new APProfile(
              new APConstraints()
                  .withVelocity(maxLinearSpeed)
                  .withAcceleration(maxLinearAcceleration)
                  .withJerk(2.0))
          .withErrorXY(Inches.of(3))
          .withErrorTheta(Degrees.of(3))
          .withBeelineRadius(Inches.of(5));

  // PathPlanner configuration
  public static final RobotConfig ppConfig =
      new RobotConfig(
          mass,
          moi,
          new com.pathplanner.lib.config.ModuleConfig(
              wheelRadius, maxLinearSpeed, 1, DCMotor.getKrakenX60Foc(1).withReduction(0), 0, 1),
          moduleTranslations);

  public static final String[] moduleNames = {
    "module 0 (Front Left)",
    "module 1 (Front Right)",
    "module 2 (Back Left)",
    "module 3 (Back Right)"
  };

  public static final ModuleConfig[] moduleConfigsComp = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(1)
        .turnMotorId(2)
        .encoderId(15)
        .encoderOffset(Rotation2d.fromRotations(0.267333984375))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(4)
        .encoderId(16)
        .encoderOffset(Rotation2d.fromRotations(0.18896484375))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(5)
        .turnMotorId(6)
        .encoderId(17)
        .encoderOffset(Rotation2d.fromRotations(-0.388671875))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(7)
        .turnMotorId(8)
        .encoderId(18)
        .encoderOffset(Rotation2d.fromRotations(0.471435546875))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // Spare
    // This should only be used at a competition if the 5th module is needed
    // To use this module modify the ecoder offest and swap the index used in RobotContainer
    ModuleConfig.builder()
        .driveMotorId(13)
        .turnMotorId(14)
        .encoderId(15)
        // Modify encoder offset using the values below
        // (FL +0) (FR +90) (BL +270) (BR +180)
        // TODO: verify these numbers are correct
        .encoderOffset(Rotation2d.fromDegrees(Units.rotationsToDegrees(0.0) + 0.0))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static final ModuleConfig[] moduleConfigsDev = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(1)
        .turnMotorId(2)
        .encoderId(3)
        .encoderOffset(Rotation2d.fromDegrees(0.15234375))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(4)
        .turnMotorId(5)
        .encoderId(6)
        .encoderOffset(Rotation2d.fromDegrees(-0.4873046875))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(7)
        .turnMotorId(8)
        .encoderId(9)
        .encoderOffset(Rotation2d.fromDegrees(-0.219482421875))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(10)
        .turnMotorId(11)
        .encoderId(12)
        .encoderOffset(Rotation2d.fromDegrees(0.17236328125))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static class PigeonConstants {
    public static final int id = 50;
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
