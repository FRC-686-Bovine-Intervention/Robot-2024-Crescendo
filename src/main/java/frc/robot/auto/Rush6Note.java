package frc.robot.auto;

import java.util.List;

import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.StartPosition;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.note.NoteVision;
import frc.robot.util.AllianceFlipUtil;

public class Rush6Note extends AutoRoutine {
    public Rush6Note(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public Rush6Note(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super("Rush 6 Note",
            List.of(),
            () -> {
                var startToSpike = AutoPaths.loadPath(String.format(AutoPaths.startToSpike, "Amp"));
                var ampSpikeToCenter = AutoPaths.loadPath("Amp Spike To Center");
                var centerToAmpWingStop = AutoPaths.loadPath("Center to Amp Wing With Stop");
                var ampWingToCenterSpike = AutoPaths.loadPath("Amp Wing to Center Spike");
                var centerSpikeToPodiumSpike = AutoPaths.loadPath("Center Spike to Podium Spike");

                var preloadShot = AllianceFlipUtil.apply(startToSpike.getPoint(startToSpike.numPoints() - 1).position);
                var ampSpikeShot = preloadShot;
                var centerShot1 = AllianceFlipUtil.apply(centerToAmpWingStop.getPoint(centerToAmpWingStop.numPoints() - 1).position);
                var centerShot2 = centerShot1;
                var centerSpikeShot = AllianceFlipUtil.apply(ampWingToCenterSpike.getPoint(centerSpikeToPodiumSpike.numPoints() - 1).position);
                var podiumSpikeShot = AllianceFlipUtil.apply(centerSpikeToPodiumSpike.getPoint(centerSpikeToPodiumSpike.numPoints() - 1).position);


                // Shoot preload and start driving to spike note
                // Intake spike note and (preemptive?) move to center
                // AutoIntake with sweep
                // Drive back to wing and shoot
                // AutoIntake with sweep
                // Drive back to wing and continue to center spike
                return AutoCommons.setOdometryFlipped(StartPosition.Amp.startPose, drive)
                    .andThen(
                        AutoCommons.shootWhenReady(preloadShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            AutoCommons.autoAim(preloadShot, shooter, pivot, drive.rotationalSubsystem),
                            AutoCommons.followPathFlipped(startToSpike, drive.translationSubsystem)
                        ),
                        AutoCommons.shootWhenReady(ampSpikeShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAim(ampSpikeShot, shooter, pivot, drive.rotationalSubsystem)
                        ),
                        AutoCommons.shootWhenReady(centerShot1, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            AutoCommons.autoAim(centerShot1, shooter, pivot),
                            AutoCommons.followPathFlipped(ampSpikeToCenter, drive)
                            .onlyWhile(() -> !noteVision.hasTarget())
                            .andThen(
                                intake.intake(drive::getChassisSpeeds)
                                .deadlineWith(
                                    noteVision.autoIntake(() -> 1.5, drive, intake)
                                ),
                                AutoCommons.followPathFlipped(centerToAmpWingStop, drive)
                            )
                        ),
                        AutoCommons.shootWhenReady(centerShot2, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            AutoCommons.autoAim(centerShot2, shooter, pivot),
                            AutoCommons.followPathFlipped(ampSpikeToCenter, drive)
                            .onlyWhile(() -> !noteVision.hasTarget())
                            .andThen(
                                intake.intake(drive::getChassisSpeeds)
                                .deadlineWith(
                                    noteVision.autoIntake(() -> 1.5, drive, intake)
                                ),
                                AutoCommons.followPathFlipped(centerToAmpWingStop, drive)
                            )
                        ),
                        AutoCommons.shootWhenReady(centerSpikeShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            AutoCommons.autoAim(centerSpikeShot, shooter, pivot, drive.rotationalSubsystem),
                            AutoCommons.followPathFlipped(ampWingToCenterSpike, drive.translationSubsystem)
                            .alongWith(
                                intake.intake(drive::getChassisSpeeds)
                            )
                        ),
                        AutoCommons.shootWhenReady(podiumSpikeShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            AutoCommons.autoAim(podiumSpikeShot, shooter, pivot, drive.rotationalSubsystem),
                            AutoCommons.followPathFlipped(centerSpikeToPodiumSpike, drive.translationSubsystem)
                            .alongWith(
                                intake.intake(drive::getChassisSpeeds)
                            )
                        )
                    )
                ;
            }
        );
    }
}
