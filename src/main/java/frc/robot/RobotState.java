package frc.robot;

import java.nio.ByteBuffer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;

public class RobotState {
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    private SwerveDrivePoseEstimator poseEstimator;

    public void initializePoseEstimator(
            SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters) {
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyroAngle, modulePositions, initialPoseMeters);
    }

    public void addDriveMeasurement(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        poseEstimator.update(rotation, modulePositions);
    }

    public void addVisionMeasurement(Pose2d pose, Matrix<N3, N1> stdDevs, double timestamp) {
        poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void log() {
        Logger.recordOutput("Odometry/Robot", getPose());
        Logger.recordOutput("AimingParameters", aimingParameters);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d fieldToVehicle) {
        poseEstimator.resetPosition(rotation, modulePositions, fieldToVehicle);
    }

    public AimingParameters aimingParameters = AimingParameters.from(new Translation2d());

    private static LoggedTunableNumber lookaheadSeconds = new LoggedTunableNumber("Aiming/Lookahead Seconds", 0.35);

    public static record AimingParameters (
        Pose2d drivePose,
        ChassisSpeeds chassisSpeeds,
        double effectiveDistance,
        double pivotAltitude,
        double targetShooterSpeed,
        double minimumShooterSpeed
    ) implements StructSerializable {
        public static AimingParameters from(Drive drive) {
            return from(drive.getPose().getTranslation(), drive.getFieldRelativeSpeeds());
        }
        public static AimingParameters from(Translation2d robotPos) {
            return from(robotPos, new ChassisSpeeds());
        }
        public static AimingParameters from(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed) {
            return from(robotPos, fieldRelativeSpeed, AllianceFlipUtil.apply(FieldConstants.speakerAimPoint));
        }
        public static AimingParameters from(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed, Translation2d aimPoint) {
            var predictedRobotPos = robotPos.plus(new Translation2d(fieldRelativeSpeed.vxMetersPerSecond, fieldRelativeSpeed.vyMetersPerSecond).times(lookaheadSeconds.get()));
            // var velocityTowardsSpeaker = robotPos.minus(aimPoint).toVector().unit().dot(VecBuilder.fill(fieldRelativeSpeed.vxMetersPerSecond, fieldRelativeSpeed.vyMetersPerSecond));
            // var timeToAimPoint = robotPos.getDistance(aimPoint) / (ShooterConstants.exitVelocity + velocityTowardsSpeaker);
            // var chassisOffset = fieldRelativeSpeed.times(timeToAimPoint);
            // var translationalOffset = new Translation2d(chassisOffset.vxMetersPerSecond, chassisOffset.vyMetersPerSecond);
            // var pointTo = aimPoint.minus(translationalOffset);
            var driveAzimuth = aimPoint.minus(predictedRobotPos).getAngle();
            var predictedDistToAimPoint = aimPoint.getDistance(predictedRobotPos);
            return new AimingParameters(
                new Pose2d(robotPos, driveAzimuth),
                new ChassisSpeeds(fieldRelativeSpeed.vxMetersPerSecond, fieldRelativeSpeed.vyMetersPerSecond, 0),
                predictedDistToAimPoint,
                ShooterConstants.pivotAltitude.get(predictedDistToAimPoint),
                ShooterConstants.targetShooterSpeed.get(predictedDistToAimPoint),
                ShooterConstants.minimumShooterSpeed.get(predictedDistToAimPoint)
            );
        }

        public static final AimParametersStruct struct = new AimParametersStruct();
        public static class AimParametersStruct implements Struct<AimingParameters> {
            @Override
            public Class<AimingParameters> getTypeClass() {
                return AimingParameters.class;
            }

            @Override
            public String getTypeString() {
                return "struct:AimingParameters";
            }

            @Override
            public int getSize() {
                return 
                    Pose2d.struct.getSize() * 1 + 
                    ChassisSpeeds.struct.getSize() * 1 + 
                    Transform3d.struct.getSize() * 1 + 
                    kSizeDouble * 4;
            }

            @Override
            public String getSchema() {
                return "Pose2d RobotPose;ChassisSpeeds ChassisSpeeds;double EffectiveDistance;double PivotAltitude;double TargetShooterSpeed;double MinimumShooterSpeed;Transform3d RobotToPivot";
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct[]{Pose2d.struct, ChassisSpeeds.struct, Transform3d.struct};
            }

            @Override
            public AimingParameters unpack(ByteBuffer bb) {
                return new AimingParameters(
                    Pose2d.struct.unpack(bb),
                    ChassisSpeeds.struct.unpack(bb),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble(),
                    bb.getDouble()
                );
            }

            @Override
            public void pack(ByteBuffer bb, AimingParameters value) {
                Pose2d.struct.pack(bb, value.drivePose);
                ChassisSpeeds.struct.pack(bb, value.chassisSpeeds);
                bb.putDouble(value.effectiveDistance);
                bb.putDouble(value.pivotAltitude);
                bb.putDouble(value.targetShooterSpeed);
                bb.putDouble(value.minimumShooterSpeed);
                Transform3d.struct.pack(bb, Pivot.getRobotToPivot(Units.degreesToRadians(value.pivotAltitude)));
            }
        }
    }
}
