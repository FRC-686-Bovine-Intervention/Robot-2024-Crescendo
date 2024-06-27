package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.MathExtraUtil;

public class AimingParameters {
    private static Pose2d shotPose;
    private static ChassisSpeeds shotSpeeds;
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
    public static void setFrom(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed, Translation2d aimPoint) {
        calculate(robotPos, fieldRelativeSpeed, aimPoint);
    }

    private static final LoggedTunableMeasure<Time> lookaheadTime = new LoggedTunableMeasure<>("Aiming/Lookahead Seconds", Seconds.of(0.35));
    
    private static void calculate(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed, Translation2d aimPoint) {
        var predictedRobotPos = robotPos.plus(MathExtraUtil.translationFromSpeeds(fieldRelativeSpeed)).times(lookaheadTime.in(Seconds));
        // var velocityTowardsSpeaker = robotPos.minus(aimPoint).toVector().unit().dot(VecBuilder.fill(fieldRelativeSpeed.vxMetersPerSecond, fieldRelativeSpeed.vyMetersPerSecond));
        // var timeToAimPoint = robotPos.getDistance(aimPoint) / (ShooterConstants.exitVelocity + velocityTowardsSpeaker);
        // var chassisOffset = fieldRelativeSpeed.times(timeToAimPoint);
        // var translationalOffset = new Translation2d(chassisOffset.vxMetersPerSecond, chassisOffset.vyMetersPerSecond);
        // var pointTo = aimPoint.minus(translationalOffset);
        var driveAzimuth = aimPoint.minus(predictedRobotPos).getAngle();
        var predictedDistToAimPoint = aimPoint.getDistance(predictedRobotPos);
        AimingParameters.shotPose = new Pose2d(robotPos, driveAzimuth);
        AimingParameters.shotSpeeds = new ChassisSpeeds(fieldRelativeSpeed.vxMetersPerSecond, fieldRelativeSpeed.vyMetersPerSecond, 0);
        AimingParameters.pivotAltitude.mut_setMagnitude(AimingConstants.pivotAltitude.get(predictedDistToAimPoint));
        AimingParameters.targetShooterSpeed.mut_setMagnitude(AimingConstants.targetShooterSpeed.get(predictedDistToAimPoint));
        AimingParameters.minimumShooterSpeed.mut_setMagnitude(AimingConstants.minimumShooterSpeed.get(predictedDistToAimPoint));
        Logger.recordOutput("AimingParameters/Shot Pose", shotPose());
        Logger.recordOutput("AimingParameters/Shot Speeds", shotSpeeds());
        Logger.recordOutput("AimingParameters/Pivot Altitude", pivotAltitude());
        Logger.recordOutput("AimingParameters/Target Shooter Speed", targetShooterSpeed());
        Logger.recordOutput("AimingParameters/Minimum Shooter Speed", minimumShooterSpeed());
        Logger.recordOutput("AimingParameters/Predicted Pose", new Pose2d(predictedRobotPos, driveAzimuth));
        Logger.recordOutput("AimingParameters/Effective Distance", AimingParameters.shotPose);
    }
}
