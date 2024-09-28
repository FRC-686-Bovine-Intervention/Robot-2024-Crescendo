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
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AimingParameters;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.Rollers;
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
        public Map.Entry<String, StartPosition> toEntry() {
            return Map.entry(this.name(), this);
        }
    }

    public static enum CenterNote {
        Note1,
        Note2,
        Note3,
        Note4,
        Note5,
        ;
        public Map.Entry<String, CenterNote> toEntry() {
            return Map.entry(this.name(), this);
        }
    }

    public static Command setOdometryFlipped(Pose2d pose, Drive drive) {
        return Commands.runOnce(() -> RobotState.getInstance().setPose(drive.getGyroRotation(), drive.getModulePositions(), AllianceFlipUtil.apply(pose)));
    }

    public static Command followPathFlipped(PathPlannerPath path, Drive drive) {
        return new FollowPathHolonomic(path, drive::getPose, drive::getRobotRelativeSpeeds, drive::driveVelocity, Drive.autoConfigSup.get(), AllianceFlipUtil::shouldFlip, drive.translationSubsystem, drive.rotationalSubsystem)
        .deadlineWith(Commands.startEnd(
            () -> Logger.recordOutput("Autonomous/Goal Pose", new Pose2d(getLastPoint(path), path.getGoalEndState().getRotation())),
            () -> Logger.recordOutput("Autonomous/Goal Pose", (Pose2d)null)
        ));
    }
    public static Command followPathFlipped(PathPlannerPath path, Drive.Translational drive) {
        return new FollowPathHolonomic(path, drive.drive::getPose, drive.drive::getRobotRelativeSpeeds, drive::driveVelocity, Drive.autoConfigSup.get(), AllianceFlipUtil::shouldFlip, drive)
        .deadlineWith(Commands.startEnd(
            () -> Logger.recordOutput("Autonomous/Goal Pose", new Pose2d(getLastPoint(path), path.getGoalEndState().getRotation())),
            () -> Logger.recordOutput("Autonomous/Goal Pose", (Pose2d)null)
        ));
    }

    public static Command shootWhenReady(double angularTolerance, Drive drive, Shooter shooter, Pivot pivot, Rollers rollers) {
        BooleanSupplier condition = () -> {
            var shooterReady = shooter.readyToAutoShoot.getAsBoolean();
            var pivotReady = pivot.atPos.getAsBoolean();
            var poseReady = MathExtraUtil.isNear(AimingParameters.shotPose(), drive.getPose(), 0.75, Units.degreesToRadians(angularTolerance));
            var speedReady = MathExtraUtil.isNear(AimingParameters.shotSpeeds(), drive.getFieldRelativeSpeeds(), 0.75, 1);

            Logger.recordOutput("DEBUG/Shooter Ready", shooterReady);
            Logger.recordOutput("DEBUG/Pivot Ready", pivotReady);
            Logger.recordOutput("DEBUG/Pose Ready", poseReady);
            Logger.recordOutput("DEBUG/Speed Ready", speedReady);

            return shooterReady && pivotReady && poseReady && speedReady;
        };
        return rollers.kicker.kick().asProxy().onlyWhile(condition).onlyIf(condition).repeatedly().until(rollers::noteExited);
    }

    public static Command autoAim(Drive.Rotational rotation) {
        return rotation.pidControlledHeading(() -> Optional.of(AimingParameters.shotPose().getRotation()));
    }
    public static Command autoAim(Shooter shooter) {
        return shooter.aimWithAutoShoot();
    }
    public static Command autoAim(Pivot pivot) {
        return pivot.aim();
    }
    public static Command autoAim(Shooter shooter, Pivot pivot) {
        return autoAim(shooter).alongWith(autoAim(pivot));
    }
    public static Command autoAim(Shooter shooter, Pivot pivot, Drive.Rotational rotation) {
        return autoAim(shooter, pivot).alongWith(autoAim(rotation));
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

    public static Command preload(Translation2d startPos, Drive drive, Shooter shooter, Pivot pivot, Rollers rollers) {
        var shotPos = AllianceFlipUtil.apply(startPos);
        return 
            AutoCommons.shootWhenReady(10, drive, shooter, pivot, rollers)
            .deadlineWith(
                AutoCommons.autoAim(shooter, pivot, drive.rotationalSubsystem).withName("Aim from Preload: " + shotPos.toString()).asProxy()
            )
            .beforeStarting(() -> AimingParameters.setFrom(shotPos))
        ;
    }

    public static void aimingFromPath(double samplePoint, PathPlannerPath path) {
        var traj = path.getTrajectory(new ChassisSpeeds(), new Rotation2d());
        System.out.println("YO IDIOT                      " + samplePoint * traj.getTotalTimeSeconds());
        System.out.println("YO IDIOT                      " + traj.getTotalTimeSeconds());
        var sampleState = traj.sample(samplePoint * traj.getTotalTimeSeconds());
        var velo = MathExtraUtil.vectorFromRotation(sampleState.heading).times(sampleState.velocityMps);
        AimingParameters.setFrom(sampleState.positionMeters, new ChassisSpeeds(velo.get(0), velo.get(1), 0));
    }

    public static Command spikeNoteSOTM(PathPlannerPath toSpike, double samplePoint, Drive drive, Shooter shooter, Pivot pivot, Rollers rollers) {
        var pathName = AutoPaths.getName(toSpike);
        return
            AutoCommons.shootWhenReady(10, drive, shooter, pivot, rollers)
            .deadlineWith(
                rollers.intake.intake().asProxy(),
                AutoCommons.autoAim(shooter, pivot, drive.rotationalSubsystem).withName("Aim Path: " + pathName).asProxy(),
                AutoCommons.followPathFlipped(toSpike, drive.translationSubsystem).withName("Aim Path: " + pathName).asProxy()
            )
            .withTimeout(3)
            .beforeStarting(() -> aimingFromPath(samplePoint, toSpike))
        ;
    }
    public static Command spikeNote(PathPlannerPath toSpike, Drive drive, Shooter shooter, Pivot pivot, Rollers rollers) {
        var shotPos = getLastPoint(toSpike);
        var pathName = AutoPaths.getName(toSpike);
        return
            AutoCommons.shootWhenReady(10, drive, shooter, pivot, rollers)
            .deadlineWith(
                rollers.intake.intake().asProxy(),
                AutoCommons.autoAim(shooter, pivot, drive.rotationalSubsystem).withName("Aim Path: " + pathName).asProxy(),
                AutoCommons.followPathFlipped(toSpike, drive.translationSubsystem).withName("Follow Path: " + pathName).asProxy()
            )
            .withTimeout(3)
            .beforeStarting(() -> AimingParameters.setFrom(shotPos))
        ;
    }
    public static Command spikeNote(PathPlannerPath toSpike, Rotation2d wiggleAngle, Drive drive, Shooter shooter, Pivot pivot, Rollers rollers) {
        var shotPos = getLastPoint(toSpike);
        var pathName = AutoPaths.getName(toSpike);
        return
            AutoCommons.shootWhenReady(10, drive, shooter, pivot, rollers)
            .deadlineWith(
                rollers.intake.intake().asProxy(),
                AutoCommons.autoAim(shooter, pivot).withName("Aim Path: " + pathName).asProxy(),
                AutoCommons.followPathFlipped(toSpike, drive.translationSubsystem).withName("Follow Path: " + pathName).asProxy(),
                drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(AllianceFlipUtil.apply(wiggleAngle))).withName("Wiggle: " + pathName).asProxy()
                .onlyWhile(rollers::noNote)
                .andThen(
                    AutoCommons.autoAim(drive.rotationalSubsystem).withName("Aim Path: " + pathName).asProxy()
                )
            )
            .withTimeout(4)
            .beforeStarting(() -> AimingParameters.setFrom(shotPos))
        ;
    }

    public static Command centerNote(PathPlannerPath toCenterLine, PathPlannerPath defaultReturn, PathPlannerPath altReturn, Drive drive, Shooter shooter, Pivot pivot, Rollers rollers, NoteVision noteVision) {
        var centerName = AutoPaths.getName(toCenterLine);
        var defaultShot = getLastPoint(defaultReturn);
        var defaultName = AutoPaths.getName(defaultReturn);
        var altShot = getLastPoint(altReturn);
        var altName = AutoPaths.getName(altReturn);
        var defaultStartPoint = getFirstPoint(defaultReturn);
        var altStartPoint = getFirstPoint(altReturn);
        BooleanSupplier isDefault = () -> drive.getPose().getTranslation().nearest(List.of(defaultStartPoint, altStartPoint)).equals(defaultStartPoint);
        return
            Commands.parallel(
                Commands.runOnce(() -> AimingParameters.setFrom(defaultShot)),
                Commands.runOnce(noteVision::clearMemory),
                
                AutoCommons.followPathFlipped(toCenterLine, drive).withName("Follow Path: " + centerName).asProxy()
                .until(noteVision::hasTarget)
                .andThen(
                    rollers.intake.intake().asProxy()
                    .raceWith(
                        noteVision.autoIntake(() -> 2, rollers::noNote, drive).withName("Auto Intake").asProxy()
                    )
                    .withTimeout(3)
                )
                .onlyWhile(rollers::noNote)
                .deadlineWith(
                    isStagePath(toCenterLine) ? (
                        AutoCommons.autoAim(shooter).withName("Aim Path: " + defaultName).asProxy()
                    ) : (
                        AutoCommons.autoAim(shooter, pivot).withName("Aim Path: " + defaultName).asProxy()
                    )
                )
                .andThen(
                    Commands.either((
                        AutoCommons.shootWhenReady(3, drive, shooter, pivot, rollers)
                        .deadlineWith(
                            AutoCommons.autoAim(shooter).withName("Aim Path: " + defaultName).asProxy(),
                            returnFromCenter(defaultReturn, drive, shooter, pivot)
                        )
                    ),(
                        AutoCommons.shootWhenReady(3, drive, shooter, pivot, rollers)
                        .deadlineWith(
                            AutoCommons.autoAim(shooter).withName("Aim Path: " + altName).asProxy(),
                            returnFromCenter(altReturn, drive, shooter, pivot)
                        )
                        .beforeStarting(() -> AimingParameters.setFrom(altShot))
                    ),
                    isDefault
                )
                .onlyIf(() -> !rollers.noNote())
                )
            )
        ;
    }

    private static Command returnFromCenter(PathPlannerPath path, Drive drive, Shooter shooter, Pivot pivot) {
        var pathName = AutoPaths.getName(path);
        return
            AutoCommons.autoAim(drive.rotationalSubsystem).withName("Aim Path: " + pathName).asProxy()
            .alongWith(
                isStagePath(path) ? (
                    AutoCommons.followPathFlipped(path, drive.translationSubsystem).withName("Follow Path: " + pathName).asProxy()
                    .andThen(AutoCommons.autoAim(pivot).withName("Aim Path: " + pathName).asProxy())
                ) : (
                    AutoCommons.followPathFlipped(path, drive.translationSubsystem).withName("Follow Path: " + pathName).asProxy()
                    .alongWith(AutoCommons.autoAim(pivot).withName("Aim Path: " + pathName).asProxy())
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
            PathPlannerLogging.setLogActivePathCallback((path) -> Logger.recordOutput("Autonomous/Path", path.toArray(Pose2d[]::new)));
            PathPlannerLogging.setLogTargetPoseCallback((target) -> Logger.recordOutput("Autonomous/Target Pose", target));
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

        public static String getName(PathPlannerPath path) {
            return loadedPaths.entrySet().stream().filter((e) -> e.getValue() == path).map((e) -> e.getKey()).findAny().orElse("Unknown Path");
        }
    }
}
