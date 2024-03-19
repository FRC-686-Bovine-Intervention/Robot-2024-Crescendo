package frc.robot.util;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MathExtraUtil {
    public static double average(double... a) {
        return Arrays.stream(a).average().orElse(0);
    }

    public static boolean isNear(Pose2d expected, Pose2d actual, double linearTolerance, double angularTolerance) {
        var bol = isNear(expected.getTranslation(), actual.getTranslation(), linearTolerance) && isNear(expected.getRotation(), actual.getRotation(), angularTolerance);
        return bol;
    }
    public static boolean isNear(Translation2d expected, Translation2d actual, double tolerance) {
        return actual.getDistance(expected) <= tolerance;
    }
    public static boolean isNear(Rotation2d expected, Rotation2d actual, double tolerance) {
        return Math.abs(actual.minus(expected).getRadians()) <= tolerance;
    }
    public static boolean isNear(ChassisSpeeds expected, ChassisSpeeds actual, double linearTolerance, double angularTolerance) {
        var bol = isNear(new Translation2d(expected.vxMetersPerSecond, expected.vyMetersPerSecond), new Translation2d(actual.vxMetersPerSecond, actual.vyMetersPerSecond), linearTolerance) && MathUtil.isNear(expected.omegaRadiansPerSecond, actual.omegaRadiansPerSecond, angularTolerance);
        return bol;
    }
}
