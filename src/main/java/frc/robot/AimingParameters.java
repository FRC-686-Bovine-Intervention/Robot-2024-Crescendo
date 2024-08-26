package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import frc.robot.Constants.AimingConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.MathExtraUtil;

public class AimingParameters {
    private static Translation3d aimPoint = new Translation3d();
    private static Pose2d shotPose = new Pose2d(1,0,new Rotation2d());
    private static ChassisSpeeds shotSpeeds = new ChassisSpeeds();
    private static final MutableMeasure<Distance> effectiveDistance = MutableMeasure.zero(Meters);
    private static final MutableMeasure<Angle> pivotAltitude = MutableMeasure.zero(Degrees);
    private static final MutableMeasure<Velocity<Distance>> targetShooterSpeed = MutableMeasure.zero(MetersPerSecond);
    private static final MutableMeasure<Velocity<Distance>> minimumShooterSpeed = MutableMeasure.zero(MetersPerSecond);

    public static Pose2d shotPose() {
        return shotPose;
    }
    public static ChassisSpeeds shotSpeeds() {
        return shotSpeeds;
    }
    public static Measure<Angle> pivotAltitude() {
        return pivotAltitude;
    }
    public static Measure<Velocity<Distance>> targetShooterSpeed() {
        return targetShooterSpeed;
    }
    public static Measure<Velocity<Distance>> minimumShooterSpeed() {
        return minimumShooterSpeed;
    }
    
    public static void setFrom(Drive drive) {
        setFrom(drive.getPose().getTranslation(), drive.getFieldRelativeSpeeds());
    }
    public static void setFrom(Translation2d robotPos) {
        setFrom(robotPos, new ChassisSpeeds());
    }
    public static void setFrom(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed) {
        setFrom(robotPos, fieldRelativeSpeed, AllianceFlipUtil.apply(FieldConstants.speakerAimPoint));
    }
    public static void setFrom(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed, Translation3d aimPoint) {
        calculate(robotPos, fieldRelativeSpeed, aimPoint);
    }

    private static final LoggedTunableMeasure<Time> lookaheadTime = new LoggedTunableMeasure<>("Aiming/Lookahead Seconds", Seconds.of(0.035));
    private static final LoggedTunableMeasure<Distance> azimuthTolerance = new LoggedTunableMeasure<>("Aiming/Tolerance/Azimuth", Centimeters.of(100));
    private static final LoggedTunableMeasure<Distance> altitudeTolerance = new LoggedTunableMeasure<>("Aiming/Tolerance/Altitude", Centimeters.of(46));

    public static final boolean withinAzimuthTolerance(Pose2d robotPose) {
        var blueRobotPose = AllianceFlipUtil.apply(robotPose);
        var blueAimPoint = AllianceFlipUtil.apply(AimingParameters.aimPoint);
        var aimPointToRobot = blueRobotPose.getTranslation().minus(blueAimPoint.toTranslation2d()).getAngle();
        var horizontalTolerance = Math.abs(aimPointToRobot.getCos()) * AimingParameters.azimuthTolerance.in(Meters) * 0.5;
        var azimuthTolerance = Math.atan2(horizontalTolerance, effectiveDistance.in(Meters));
        var result = MathExtraUtil.isNear(AimingParameters.shotPose().getRotation(), robotPose.getRotation(), azimuthTolerance);
        Logger.recordOutput("AimingTolerance/Azimuth", result);
        return result;
    }

    public static final boolean withinAltitudeTolerance(Measure<Angle> altitude) {
        var pivotDist = effectiveDistance.in(Meters) - Pivot.robotToPivotTranslation.getX();
        var targetHeight = aimPoint.getZ() - Pivot.robotToPivotTranslation.getZ();
        var pivotToTargetDist = Math.hypot(pivotDist, targetHeight);
        var verticalTolerance = Math.abs(targetHeight / pivotToTargetDist) * AimingParameters.altitudeTolerance.in(Meters) * 0.5;
        var altitudeTolerance = Math.atan2(verticalTolerance, pivotToTargetDist - (targetHeight / pivotDist * verticalTolerance));
        Logger.recordOutput("AimingTolerance/pivotDist", pivotDist);
        Logger.recordOutput("AimingTolerance/pivotToTargetDist", pivotToTargetDist);
        Logger.recordOutput("AimingTolerance/verticalTolerance", verticalTolerance);
        Logger.recordOutput("AimingTolerance/altitudeTolerance", altitudeTolerance);
        var result = MathExtraUtil.isNear(AimingParameters.pivotAltitude(), altitude, Radians.of(altitudeTolerance));
        Logger.recordOutput("AimingTolerance/Altitude", result);
        return result;
    }
    
    private static void calculate(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed, Translation3d aimPoint) {
        var predictedRobotPos = robotPos.plus(MathExtraUtil.translationFromSpeeds(fieldRelativeSpeed).times(lookaheadTime.in(Seconds)));
        // var velocityTowardsSpeaker = robotPos.minus(aimPoint).toVector().unit().dot(VecBuilder.fill(fieldRelativeSpeed.vxMetersPerSecond, fieldRelativeSpeed.vyMetersPerSecond));
        // var timeToAimPoint = robotPos.getDistance(aimPoint) / (ShooterConstants.exitVelocity + velocityTowardsSpeaker);
        // var chassisOffset = fieldRelativeSpeed.times(timeToAimPoint);
        // var translationalOffset = new Translation2d(chassisOffset.vxMetersPerSecond, chassisOffset.vyMetersPerSecond);
        // var pointTo = aimPoint.minus(translationalOffset);
        var driveAzimuth = aimPoint.toTranslation2d().minus(predictedRobotPos).getAngle();
        var predictedDistToAimPoint = aimPoint.toTranslation2d().getDistance(predictedRobotPos);
        AimingParameters.aimPoint = aimPoint;
        AimingParameters.shotPose = new Pose2d(robotPos, driveAzimuth);
        AimingParameters.shotSpeeds = new ChassisSpeeds(fieldRelativeSpeed.vxMetersPerSecond, fieldRelativeSpeed.vyMetersPerSecond, 0);
        AimingParameters.effectiveDistance.mut_setMagnitude(predictedDistToAimPoint);
        AimingParameters.pivotAltitude.mut_setMagnitude(AimingConstants.pivotAltitude.get(predictedDistToAimPoint));
        AimingParameters.targetShooterSpeed.mut_setMagnitude(AimingConstants.targetShooterSpeed.get(predictedDistToAimPoint));
        AimingParameters.minimumShooterSpeed.mut_setMagnitude(AimingConstants.minimumShooterSpeed.get(predictedDistToAimPoint));
        Logger.recordOutput("AimingParameters/Aim Point", AimingParameters.aimPoint);
        Logger.recordOutput("AimingParameters/Shot Pose", shotPose());
        Logger.recordOutput("AimingParameters/Shot Speeds", shotSpeeds());
        Logger.recordOutput("AimingParameters/Pivot Altitude", pivotAltitude());
        Logger.recordOutput("AimingParameters/Target Shooter Speed", targetShooterSpeed());
        Logger.recordOutput("AimingParameters/Minimum Shooter Speed", minimumShooterSpeed());
        Logger.recordOutput("AimingParameters/Predicted Pose", new Pose2d(predictedRobotPos, driveAzimuth));
        Logger.recordOutput("AimingParameters/Effective Distance", effectiveDistance);
    }
}
