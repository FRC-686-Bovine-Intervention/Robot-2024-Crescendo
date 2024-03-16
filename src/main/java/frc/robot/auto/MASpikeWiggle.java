package frc.robot.auto;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.StartPosition;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.note.NoteVision;
import frc.robot.util.AllianceFlipUtil;

public class MASpikeWiggle extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> new StartPosition[]{StartPosition.Amp});

    public MASpikeWiggle(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public MASpikeWiggle(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super("MA Spike Wiggle",
            List.of(),
            () -> {
                PathPlannerPath startToSpike = AutoPaths.loadPath(String.format(AutoPaths.startToSpike, "Amp"));
                PathPlannerPath ampSpikeToCenterSpike = AutoPaths.loadPath("MASW Amp Spike to Center Spike");
                PathPlannerPath centerSpikeToPodiumSpike = AutoPaths.loadPath("MASW Center Spike to Podium Spike");
                // PathPlannerPath podiumSpikeToWing = AutoPaths.loadPath("MASW Podium Spike to Amp Wing");

                var preloadShot = AllianceFlipUtil.apply(startPosition.getResponse().startPose.getTranslation());
                var ampSpikeShot = AllianceFlipUtil.apply(startToSpike.getPoint(startToSpike.numPoints() - 1).position);
                var centerSpikeShot = AllianceFlipUtil.apply(ampSpikeToCenterSpike.getPoint(startToSpike.numPoints() - 1).position);
                var podiumSpikeShot = AllianceFlipUtil.apply(centerSpikeToPodiumSpike.getPoint(startToSpike.numPoints() - 1).position);

                return AutoCommons.setOdometryFlipped(startPosition.getResponse().startPose, drive)
                    .andThen(
                        AutoCommons.shootWhenReady(preloadShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            AutoCommons.autoAim(preloadShot, shooter, kicker, pivot, drive.rotationalSubsystem)
                        ),
                        AutoCommons.shootWhenReady(ampSpikeShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAim(ampSpikeShot, shooter, kicker, pivot, drive.rotationalSubsystem),
                            AutoCommons.followPathFlipped(startToSpike, drive.translationSubsystem)
                        ),
                        AutoCommons.shootWhenReady(centerSpikeShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAim(centerSpikeShot, shooter, kicker, pivot),
                            AutoCommons.followPathFlipped(ampSpikeToCenterSpike, drive.translationSubsystem),
                            drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(AllianceFlipUtil.apply(Rotation2d.fromDegrees(135))))
                            .until(intake::hasNote)
                            .andThen(
                                AutoCommons.autoAim(centerSpikeShot, drive.rotationalSubsystem)
                            )
                        ),
                        AutoCommons.shootWhenReady(podiumSpikeShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAim(podiumSpikeShot, shooter, kicker, pivot),
                            AutoCommons.followPathFlipped(centerSpikeToPodiumSpike, drive.translationSubsystem),
                            drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(AllianceFlipUtil.apply(Rotation2d.fromDegrees(135))))
                            .until(intake::hasNote)
                            .andThen(
                                AutoCommons.autoAim(podiumSpikeShot, drive.rotationalSubsystem)
                            )
                        )
                        // ,
                        // AutoCommons.followPathFlipped(podiumSpikeToWing, drive)
                        // .onlyWhile(() -> !noteVision.hasTarget())
                        // .andThen(
                        //     intake.intake(drive::getChassisSpeeds)
                        //     .deadlineWith(
                        //         noteVision.autoIntake(() -> 1.5, drive, intake)
                        //     )
                        // )
                    )
                ;
            }
        );
    }
}
