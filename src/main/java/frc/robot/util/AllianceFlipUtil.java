package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class AllianceFlipUtil {
  public static enum FieldFlipType {
    CenterPointFlip,
    MirrorFlip,
  }
  public static final FieldFlipType defaultFlipType = FieldFlipType.MirrorFlip;

  public static Translation2d apply(Translation2d translation) {
    return apply(translation, defaultFlipType);
  }
  public static Translation2d apply(Translation2d translation, FieldFlipType flipType) {
    if(!shouldFlip()) return translation;
    switch(flipType) {
      default:
      case CenterPointFlip: return new Translation2d(FieldConstants.fieldLength - translation.getX(), FieldConstants.fieldWidth - translation.getY());
      case MirrorFlip:      return new Translation2d(FieldConstants.fieldLength - translation.getX(), translation.getY());
    }
  }

  public static Translation3d apply(Translation3d translation) {
    return apply(translation, defaultFlipType);
  }
  public static Translation3d apply(Translation3d translation, FieldFlipType flipType) {
    if(!shouldFlip()) return translation;
    switch(flipType) {
      default:
      case CenterPointFlip: return new Translation3d(FieldConstants.fieldLength - translation.getX(), FieldConstants.fieldWidth - translation.getY(), translation.getZ());
      case MirrorFlip:      return new Translation3d(FieldConstants.fieldLength - translation.getX(), translation.getY(), translation.getZ());
    }
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return apply(rotation, defaultFlipType);
  }
  public static Rotation2d apply(Rotation2d rotation, FieldFlipType flipType) {
    if(!shouldFlip()) return rotation;
    switch(flipType) {
      default:
      case CenterPointFlip: return rotation.rotateBy(Rotation2d.fromRotations(0.5));
      case MirrorFlip:      return new Rotation2d(-rotation.getCos(), rotation.getSin());
    }
  }

  public static Pose2d apply(Pose2d pose) {
    return apply(pose, defaultFlipType);
  }
  public static Pose2d apply(Pose2d pose, FieldFlipType flipType) {
    if(!shouldFlip()) return pose;
    return new Pose2d(apply(pose.getTranslation(), flipType), apply(pose.getRotation(), flipType));
  }

  public static ChassisSpeeds applyFieldRelative(ChassisSpeeds speeds) {
    return applyFieldRelative(speeds, defaultFlipType);
  }
  public static ChassisSpeeds applyFieldRelative(ChassisSpeeds speeds, FieldFlipType flipType) {
    if(!shouldFlip()) return speeds;
    switch (flipType) {
      default:
      case CenterPointFlip: return new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
      case MirrorFlip: return new ChassisSpeeds(-speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }
  }

  public static ChassisSpeeds applyRobotRelative(ChassisSpeeds speeds, Rotation2d robotRotation) {
    return applyRobotRelative(speeds, robotRotation, defaultFlipType);
  }
  public static ChassisSpeeds applyRobotRelative(ChassisSpeeds speeds, Rotation2d robotRotation, FieldFlipType flipType) {
    return ChassisSpeeds.fromFieldRelativeSpeeds(applyFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotRotation)), robotRotation);
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
  }
}
