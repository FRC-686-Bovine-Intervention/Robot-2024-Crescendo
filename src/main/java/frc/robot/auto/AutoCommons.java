package frc.robot.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.note.NoteVision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.MathExtraUtil;

public class AutoCommons {
    public static enum StartPosition {
        SubwooferFront(FieldConstants.subwooferFront),
        SubwooferAmp(FieldConstants.subwooferAmp),
        SubwooferSource(FieldConstants.subwooferSource),
        Amp(new Pose2d(
            new Translation2d(
                1.40,
                6.80
            ),
            Rotation2d.fromDegrees(180)
        )),
        Podium(new Pose2d(
            new Translation2d(
                1.40,
                4.20
            ),
            Rotation2d.fromDegrees(180)
        )),
        Source(new Pose2d(
            new Translation2d(
                1.40,
                3.30
            ),
            Rotation2d.fromDegrees(180)
        ))
        ;
        public final Pose2d startPose;
        StartPosition(Pose2d startPose) {
            this.startPose = startPose;
        }
    }

    public static enum Count {
        k1(1),
        k2(2),
        k3(3),
        k4(4),
        k5(5),
        k6(6),
        ;
        public final int asInt;
        Count(int asInt) {
            this.asInt = asInt;
        }
    }

    public static enum CenterNote {
        Note1,
        Note2,
        Note3,
        Note4,
        Note5,
        ;
    }

    public static enum Bool {
        Yes(true),
        No(false),
        ;
        public final boolean asBoolean;
        Bool(boolean asBoolean) {
            this.asBoolean = asBoolean;
        }
    }

    public static Command setOdometryFlipped(Pose2d pose, Drive drive) {
        return Commands.runOnce(() -> RobotState.getInstance().setPose(drive.getGyroRotation(), drive.getModulePositions(), AllianceFlipUtil.apply(pose)));
    }

    public static Command followPathFlipped(PathPlannerPath path, Drive drive) {
        return new FollowPathHolonomic(path, drive::getPose, drive::getChassisSpeeds, drive::driveVelocity, Drive.autoConfigSup.get(), AllianceFlipUtil::shouldFlip, drive.translationSubsystem, drive.rotationalSubsystem);
    }
    public static Command followPathFlipped(PathPlannerPath path, Drive.Translational drive) {
        return new FollowPathHolonomic(path, drive.drive::getPose, drive.drive::getChassisSpeeds, drive::driveVelocity, Drive.autoConfigSup.get(), AllianceFlipUtil::shouldFlip, drive);
    }

    public static Command shootWhenReady(Translation2d pos, double angularTolerance, Drive drive, Shooter shooter, Pivot pivot, Kicker kicker) {
        var FORR = getFORR(pos);
        // var dist = FORR.getNorm();
        var shootPos = new Pose2d(pos, new Rotation2d(FORR.getX(), FORR.getY()));
        BooleanSupplier condition = () -> {
            var shooterReady = shooter.readyToShoot();
            var pivotReady = pivot.readyToShoot();
            var poseReady = MathExtraUtil.isNear(shootPos, drive.getPose(), 0.75, Units.degreesToRadians(angularTolerance));
            var speedReady = MathExtraUtil.isNear(new ChassisSpeeds(), drive.getChassisSpeeds(), 0.75, 1);

            Logger.recordOutput("DEBUG/Shooter Ready", shooterReady);
            Logger.recordOutput("DEBUG/Pivot Ready", pivotReady);
            Logger.recordOutput("DEBUG/Pose Ready", poseReady);
            Logger.recordOutput("DEBUG/Speed Ready", speedReady);

            return shooterReady && pivotReady && poseReady && speedReady;
        };
        return kicker.kick().asProxy().onlyWhile(condition).onlyIf(condition).repeatedly().until(kicker::sensorFallingEdge);
        // return Commands.waitUntil(() -> 
        //     // kicker.hasNote() && 
        //     shooter.readyToShoot() && 
        //     pivot.isAtAngle(ShooterConstants.distLerp(dist, ShooterConstants.angle)) && 
        //     MathExtraUtil.isNear(shootPos, drive.getPose(), 0.75, Units.degreesToRadians(3)) && 
        //     MathExtraUtil.isNear(new ChassisSpeeds(), drive.getChassisSpeeds(), 0.5, 0.2)
        // )
        // .andThen(kicker.kick().asProxy());
    }

    private static Translation2d getFORR(Translation2d pos) {
        return AllianceFlipUtil.apply(FieldConstants.speakerAimPoint).minus(pos);
    }
    public static Command autoAim(Translation2d pos, Drive.Rotational rotation) {
        return rotation.pidControlledHeading(() -> Optional.of(getFORR(pos)).map((t) -> new Rotation2d(t.getX(), t.getY())));
    }
    public static Command autoAim(Translation2d pos, Shooter shooter) {
        return shooter.shoot(() -> getFORR(pos)).asProxy();
    }
    public static Command autoAim(Translation2d pos, Pivot pivot) {
        return pivot.autoAim(() -> getFORR(pos)).asProxy();
    }
    public static Command autoAim(Translation2d pos, Shooter shooter, Pivot pivot) {
        return autoAim(pos, shooter).alongWith(autoAim(pos, pivot));
    }
    public static Command autoAim(Translation2d pos, Shooter shooter, Pivot pivot, Drive.Rotational rotation) {
        return autoAim(pos, shooter, pivot).alongWith(autoAim(pos, rotation));
    }

    public static Translation2d getFirstPoint(PathPlannerPath path) {
        return AllianceFlipUtil.apply(path.getPoint(0).position);
    }

    public static Translation2d getLastPoint(PathPlannerPath path) {
        return AllianceFlipUtil.apply(path.getPoint(path.numPoints() - 1).position);
    }

    public static boolean isStagePath(PathPlannerPath path) {
        return AutoPaths.stagePaths.contains(path);
    }

    public static Command centerNote(PathPlannerPath toCenterLine, PathPlannerPath defaultReturn, PathPlannerPath altReturn, Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        var defaultShot = getLastPoint(defaultReturn);
        var altShot = getLastPoint(altReturn);
        var defaultStartPoint = getFirstPoint(defaultReturn);
        var altStartPoint = getFirstPoint(altReturn);
        BooleanSupplier isDefault = () -> drive.getPose().getTranslation().nearest(List.of(defaultStartPoint, altStartPoint)).equals(defaultStartPoint);
        return
            Commands.runOnce(noteVision::clearMemory)
            .andThen(
                AutoCommons.followPathFlipped(toCenterLine, drive)
                .onlyWhile(
                    () -> !noteVision.hasTarget()
                ),
                intake.intake(drive::getChassisSpeeds)
                .raceWith(
                    noteVision.autoIntake(() -> 2, drive, intake)
                )
                .withTimeout(3)
            )
            .deadlineWith(
                isStagePath(toCenterLine) ? (
                    AutoCommons.autoAim(defaultShot, shooter)
                ) : (
                    AutoCommons.autoAim(defaultShot, shooter, pivot)
                )
            )
            .andThen(
                Commands.either((
                    AutoCommons.shootWhenReady(defaultShot, 3, drive, shooter, pivot, kicker)
                    .deadlineWith(
                        isStagePath(defaultReturn) ? (
                            AutoCommons.autoAim(defaultShot, shooter)
                        ) : (
                            AutoCommons.autoAim(defaultShot, shooter, pivot)
                        ),
                        returnFromCenter(defaultReturn, drive, shooter, pivot, kicker)
                    )
                ),(
                    AutoCommons.shootWhenReady(altShot, 3, drive, shooter, pivot, kicker)
                    .deadlineWith(
                        isStagePath(altReturn) ? (
                            AutoCommons.autoAim(altShot, shooter)
                        ) : (
                            AutoCommons.autoAim(altShot, shooter, pivot)
                        ),
                        returnFromCenter(altReturn, drive, shooter, pivot, kicker)
                    )
                ),
                isDefault
            )
            .onlyIf(intake::hasNote)
            )
        ;
        // return 
        //     AutoCommons.shootWhenReady(centerShot1, 3, drive, shooter, pivot, kicker)
        //     .deadlineWith(
        //         AutoCommons.autoAim(centerShot1, shooter, kicker, pivot),
        //         Commands.runOnce(noteVision::clearMemory)
        //         .andThen(
        //             AutoCommons.followPathFlipped(spikeToCenter, drive)
        //             .onlyWhile(() -> !noteVision.hasTarget())
        //             .andThen(
        //                 intake.intake(drive::getChassisSpeeds)
        //                 .deadlineWith(
        //                     noteVision.autoIntake(() -> 2, drive, intake)
        //                 ),
        //                 AutoCommons.autoAim(centerShot1, drive.rotationalSubsystem)
        //                 .alongWith(
        //                     AutoCommons.followPathFlipped(centerToWing, drive.translationSubsystem)
        //                 )
        //             )
        //         )
        //     )
        // ;
    }

    private static Command returnFromCenter(PathPlannerPath path, Drive drive, Shooter shooter, Pivot pivot, Kicker kicker) {
        var shotPos = getLastPoint(path);
        return
            AutoCommons.autoAim(shotPos, drive.rotationalSubsystem)
            .alongWith(
                isStagePath(path) ? (
                    AutoCommons.followPathFlipped(path, drive.translationSubsystem)
                    .andThen(AutoCommons.autoAim(shotPos, pivot))
                ) : (
                    AutoCommons.followPathFlipped(path, drive.translationSubsystem)
                    .alongWith(AutoCommons.autoAim(shotPos, pivot))
                )
            )
        ;
    }

    public static class AutoPaths {
        // public static final String toCenterLine = "%s Start";
        // public static final String fromCenterLine = "%s Back";

        public static final String startToSpike = "%s Start to Spike";

        public static final List<PathPlannerPath> stagePaths = new ArrayList<>();

        private static final Map<String, PathPlannerPath> loadedPaths = new HashMap<>();
        private static boolean preloading;
        public static void preload() {
            preloading = true;
            loadPath("Amp Start to Spike");
            loadPath("MASW Amp Spike to Center Spike");
            loadPath("MASW Center Spike to Amp Spike");
            loadPath("MASW Center Spike to Podium Spike");
            loadPath("MASW Podium Spike to Center Note1");
            loadPath("MASW Podium Spike to Center Note2");
            loadPath("MASW Podium Spike to Center Note3", true);
            loadPath("MASW Podium Spike to Center Spike");
            loadPath("Podium Start to Spike");
            loadPath("R6N Amp Spike to Center Note1");
            loadPath("R6N Amp Spike to Center Note2");
            loadPath("R6N Amp Spike to Center Note3", true);
            loadPath("R6N Amp Start to Spike");
            loadPath("R6N Amp Wing to Center Note1");
            loadPath("R6N Amp Wing to Center Note2");
            loadPath("R6N Amp Wing to Center Note3", true);
            loadPath("R6N Center Note1 to Amp Wing");
            loadPath("S4N Center Note3 to Amp Wing", true);
            loadPath("S4N Center Note5 to Source Wing");
            loadPath("S4N Podium Spike to Center Note4");
            loadPath("S4N Podium Spike to Center Note5");
            loadPath("S4N Source Wing to Center Note3", true);
            loadPath("S4N Source Wing to Center Note4");
            loadPath("S4N Source Wing to Center Note5");
            preloading = false;
            System.out.println("[Init AutoPaths] Loaded paths");
        }

        public static PathPlannerPath loadPath(String name) {
            return loadPath(name, false);
        }

        public static PathPlannerPath loadPath(String name, boolean isStagePath) {
            if(loadedPaths.containsKey(name)) {
                return loadedPaths.get(name);
            } else {
                if(!preloading) new Alert("[AutoPaths] Loading \"" + name + "\" which wasn't preloaded. Please add path to AutoPaths.preload()", AlertType.WARNING).set(true);
                var path = PathPlannerPath.fromPathFile(name);
                loadedPaths.put(name, path);
                if(isStagePath) {
                    stagePaths.add(path);
                }
                return path;
            }
        }
    }
}
