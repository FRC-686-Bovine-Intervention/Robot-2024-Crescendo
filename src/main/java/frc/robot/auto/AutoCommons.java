package frc.robot.auto;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.RobotState;
import frc.robot.SuperCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.commands.FieldOrientedDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.note.NoteVision;
import frc.robot.util.AllianceFlipUtil;

public class AutoCommons {
    public static enum StartPosition {
        SubwooferFront(FieldConstants.subwooferFront),
        SubwooferAmp(FieldConstants.subwooferAmp),
        SubwooferSource(FieldConstants.subwooferSource),
        Amp(new Pose2d(
            new Translation2d(
                1.40,
                7.00
            ),
            Rotation2d.fromDegrees(180)
        )),
        Podium(new Pose2d(
            new Translation2d(
                1.40,
                4.10
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

    public static Command setOdometryFlipped(Pose2d pose, Drive drive) {
        return Commands.runOnce(() -> RobotState.getInstance().setPose(drive.getGyroRotation(), drive.getModulePositions(), AllianceFlipUtil.apply(pose)));
    }

    public static Command followPathFlipped(PathPlannerPath path, Drive drive) {
        return new FollowPathHolonomic(path, drive::getPose, drive::getChassisSpeeds, drive::driveVelocity, Drive.autoConfigSup.get(), AllianceFlipUtil::shouldFlip, drive);
    }

    public static Command autoAimAndShootWhenReady(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker) {
        return autoAimAndShootWhenReady(ChassisSpeeds::new, drive, shooter, pivot, kicker);
    }

    public static Command autoAimAndShootWhenReady(Supplier<ChassisSpeeds> translation, Drive drive, Shooter shooter, Pivot pivot, Kicker kicker) {
        return SuperCommands.autoAim(translation, drive, shooter, pivot).deadlineWith(SuperCommands.shootWhenReady(shooter, pivot, kicker));
    }

    public static Command autoAimAndFollowPath(PathPlannerPath path, Drive drive, Shooter shooter, Pivot pivot, Kicker kicker) {
        var shootAtPos = SuperCommands.autoAimShootAtPos(drive);
        var heading = FieldOrientedDrive.pidControlledHeading(
            FieldOrientedDrive.pointTo(
                () -> Optional.of(shootAtPos.get()),
                () -> RobotConstants.shooterForward
            )
        );
        return new FollowPathHolonomic(
            path,
            drive::getPose,
            drive::getChassisSpeeds,
            (pathSpeeds) -> {
                pathSpeeds.omegaRadiansPerSecond = heading.applyAsDouble(drive.getPose().getRotation(), Optional.empty());
                drive.driveVelocity(pathSpeeds);
            },
            Drive.autoConfigSup.get(),
            AllianceFlipUtil::shouldFlip,
            drive
        ).withName("a").asProxy().alongWith(
            shooter.shoot(shootAtPos).asProxy()
            .deadlineWith(
                pivot.autoAim(shootAtPos),
                SuperCommands.shootWhenReady(shooter, pivot, kicker)
            )
        );
    }

    public static Command autoAimAsIfAt(Translation2d pos, Drive drive, Shooter shooter, Pivot pivot, Kicker kicker) {
        return SuperCommands.autoAim(SuperCommands.autoAimFORR(() -> pos, ChassisSpeeds::new), ChassisSpeeds::new, drive, shooter, pivot);
    }

    public static Command autoIntake(double throttle, Drive drive, Intake intake, NoteVision noteVision) {
        return intake.intake(drive::getChassisSpeeds).asProxy()
            .deadlineWith(
                new FieldOrientedDrive(
                    drive,
                    noteVision.getAutoIntakeTransSpeed(() -> throttle).orElseGet(ChassisSpeeds::new),
                    FieldOrientedDrive.pidControlledHeading(
                        FieldOrientedDrive.pointTo(
                            noteVision.autoIntakeTargetLocation(),
                            () -> RobotConstants.intakeForward
                        ).orElse(() -> Optional.of(AllianceFlipUtil.apply(Rotation2d.fromDegrees(180))))
                    )
                )
            )
        ;
    }

    public static class AutoPaths {
        public static final String toCenterLine = "%s Start";
        public static final String fromCenterLine = "%s Back";

        public static final String startToSpike = "%s Spike";

        private static final Map<String, PathPlannerPath> loadedPaths = new HashMap<>();
        private static boolean preloading;
        public static void preload() {
            preloading = true;
            loadPath(String.format(AutoPaths.startToSpike, "Amp"));
            preloading = false;
            System.out.println("[Init AutoPaths] Loaded paths");
        }

        public static PathPlannerPath loadPath(String name) {
            if(loadedPaths.containsKey(name)) {
                return loadedPaths.get(name);
            } else {
                if(!preloading) DriverStation.reportWarning("[AutoPaths] Loading \"" + name + "\" which wasn't preloaded. Please add path to AutoPaths.preload()", false);
                var path = PathPlannerPath.fromPathFile(name);
                loadedPaths.put(name, path);
                return path;
            }
        }
    }
}
