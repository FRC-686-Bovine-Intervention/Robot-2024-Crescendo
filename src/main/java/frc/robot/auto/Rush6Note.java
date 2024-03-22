package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Commands;
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

public class Rush6Note extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> new StartPosition[]{StartPosition.Amp, StartPosition.SubwooferAmp});

    public Rush6Note(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public Rush6Note(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super("Rush 6 Note",
            List.of(startPosition),
            () -> {
                var startToSpike = AutoPaths.loadPath("R6N Amp Start to Spike");
                var ampSpikeToCenter = AutoPaths.loadPath("R6N Amp Spike to Center");
                var centerToAmpWing = AutoPaths.loadPath("R6N Center to Amp Wing");
                var ampWingToCenter = AutoPaths.loadPath("R6N Amp Wing to Center");
                // var ampWingToCenterSpike = AutoPaths.loadPath("Amp Wing to Center Spike");
                // var centerSpikeToPodiumSpike = AutoPaths.loadPath("Center Spike to Podium Spike");

                var preloadShot = AllianceFlipUtil.apply(startPosition.getResponse().startPose.getTranslation());
                var ampSpikeShot = AllianceFlipUtil.apply(startToSpike.getPoint(startToSpike.numPoints() - 1).position);
                var centerShot1 = AllianceFlipUtil.apply(centerToAmpWing.getPoint(centerToAmpWing.numPoints() - 1).position);
                var centerShot2 = centerShot1;
                // var centerSpikeShot = AllianceFlipUtil.apply(ampWingToCenterSpike.getPoint(centerSpikeToPodiumSpike.numPoints() - 1).position);
                // var podiumSpikeShot = AllianceFlipUtil.apply(centerSpikeToPodiumSpike.getPoint(centerSpikeToPodiumSpike.numPoints() - 1).position);

                return AutoCommons.setOdometryFlipped(startPosition.getResponse().startPose, drive)
                    .andThen(
                        AutoCommons.shootWhenReady(preloadShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            AutoCommons.autoAim(preloadShot, shooter, kicker, pivot, drive.rotationalSubsystem)
                        ),
                        AutoCommons.shootWhenReady(ampSpikeShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            Commands.print("[Rush6Note] Shot Preload"),
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAim(ampSpikeShot, shooter, kicker, pivot, drive.rotationalSubsystem),
                            AutoCommons.followPathFlipped(startToSpike, drive.translationSubsystem)
                        ),
                        AutoCommons.shootWhenReady(centerShot1, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            Commands.print("[Rush6Note] Shot Amp Spike"),
                            AutoCommons.autoAim(centerShot1, shooter, kicker, pivot),
                            Commands.runOnce(noteVision::clearMemory)
                            .andThen(
                                AutoCommons.followPathFlipped(ampSpikeToCenter, drive)
                                .onlyWhile(() -> !noteVision.hasTarget())
                                .andThen(
                                    intake.intake(drive::getChassisSpeeds)
                                    .deadlineWith(
                                        noteVision.autoIntake(() -> 1.5, drive, intake)
                                    ),
                                    AutoCommons.autoAim(centerShot1, drive.rotationalSubsystem)
                                    .alongWith(
                                        AutoCommons.followPathFlipped(centerToAmpWing, drive.translationSubsystem)
                                    )
                                )
                            )
                        ),
                        AutoCommons.shootWhenReady(centerShot2, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            Commands.print("[Rush6Note] Shot Center 1"),
                            AutoCommons.autoAim(centerShot2, shooter, kicker, pivot),
                            Commands.runOnce(noteVision::clearMemory)
                            .andThen(
                                AutoCommons.followPathFlipped(ampWingToCenter, drive)
                                .onlyWhile(() -> !noteVision.hasTarget())
                                .andThen(
                                    intake.intake(drive::getChassisSpeeds)
                                    .deadlineWith(
                                        noteVision.autoIntake(() -> 1.5, drive, intake)
                                    ),
                                    AutoCommons.autoAim(centerShot2, drive.rotationalSubsystem)
                                    .alongWith(
                                        AutoCommons.followPathFlipped(centerToAmpWing, drive.translationSubsystem)
                                    )
                                )
                            )
                        ),
                        Commands.print("[Rush6Note] Shot Center 2")
                        // AutoCommons.shootWhenReady(centerSpikeShot, drive, shooter, pivot, kicker)
                        // .deadlineWith(
                        //     AutoCommons.autoAim(centerSpikeShot, shooter, pivot, drive.rotationalSubsystem),
                        //     AutoCommons.followPathFlipped(ampWingToCenterSpike, drive.translationSubsystem)
                        //     .alongWith(
                        //         intake.intake(drive::getChassisSpeeds)
                        //     )
                        // ),
                        // AutoCommons.shootWhenReady(podiumSpikeShot, drive, shooter, pivot, kicker)
                        // .deadlineWith(
                        //     Commands.print("[Rush6Note] Shot Center Spike"),
                        //     // AutoCommons.autoAim(podiumSpikeShot, shooter, pivot, drive.rotationalSubsystem),
                        //     // AutoCommons.followPathFlipped(centerSpikeToPodiumSpike, drive.translationSubsystem)
                        //     // .alongWith(
                        //     //     intake.intake(drive::getChassisSpeeds)
                        //     // )
                        //     AutoCommons.autoAim(podiumSpikeShot, shooter, pivot),
                        //     Commands.runOnce(noteVision::clearMemory)
                        //     .andThen(
                        //         AutoCommons.followPathFlipped(centerSpikeToPodiumSpike, drive)
                        //         .onlyWhile(() -> !noteVision.hasTarget())
                        //         .andThen(
                        //             intake.intake(drive::getChassisSpeeds)
                        //             .deadlineWith(
                        //                 noteVision.autoIntake(() -> 2, drive, intake)
                        //             ),
                        //             AutoCommons.autoAim(podiumSpikeShot, drive.rotationalSubsystem)
                        //         )
                        //     )
                        // ),
                        // Commands.print("[Rush6Note] Shot Podium Spike")
                    )
                ;
            }
        );
    }
}
