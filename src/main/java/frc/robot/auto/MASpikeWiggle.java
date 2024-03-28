package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
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

public class MASpikeWiggle extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> new StartPosition[]{
        StartPosition.Amp,
        StartPosition.Podium
    });
    private static final AutoQuestion<Number> noteCount = new AutoQuestion<>("Note Count", Number::values);

    private static enum Number {
        k5(5),
        k4(4),
        k3(3),
        k2(2),
        k1(1),
        ;
        public final int asInt;
        Number(int asInt) {
            this.asInt = asInt;
        }
    }

    public MASpikeWiggle(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public MASpikeWiggle(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super("MA Spike Wiggle",
            List.of(startPosition, noteCount),
            () -> {
                var startPosition = MASpikeWiggle.startPosition.getResponse();
                var noteCount = MASpikeWiggle.noteCount.getResponse();
                
                var commands = new ArrayList<Command>();

                if(noteCount.asInt >= 1) {
                    var preloadShot = AllianceFlipUtil.apply(startPosition.startPose.getTranslation());
                    commands.add(
                        AutoCommons.shootWhenReady(preloadShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            AutoCommons.autoAim(preloadShot, shooter, kicker, pivot, drive.rotationalSubsystem)
                        )
                    );
                }

                if(noteCount.asInt >= 2) {
                    var startToSpike1 = AutoPaths.loadPath(
                        switch(startPosition) {
                            case Amp, SubwooferAmp -> "MASW Amp Start to Spike";
                            case Podium, SubwooferSource -> "MASW Podium Start to Spike";
                            default -> "";
                        }
                    );
                    var spike1Shot = AllianceFlipUtil.apply(startToSpike1.getPoint(startToSpike1.numPoints() - 1).position);
                    commands.add(
                        AutoCommons.shootWhenReady(spike1Shot, drive, shooter, pivot, kicker)
                        // .raceWith(
                        //     Commands.waitSeconds(2.5)
                        //     .andThen(
                        //         Commands.waitUntil(() -> 
                        //             !(
                        //                 intake.getIntakeCommand().equals(Optional.of(IntakeCommand.FEED_TO_KICKER)) || 
                        //                 kicker.hasNote()
                        //             )
                        //         )
                        //     )
                        // )
                        .deadlineWith(
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAim(spike1Shot, shooter, kicker, pivot, drive.rotationalSubsystem),
                            AutoCommons.followPathFlipped(startToSpike1, drive.translationSubsystem)
                        )
                    );
                }

                if(noteCount.asInt >= 3) {
                    var spike1ToSpike2 = AutoPaths.loadPath(
                        switch(startPosition) {
                            case Amp, SubwooferAmp -> "MASW Amp Spike to Center Spike";
                            case Podium, SubwooferSource -> "MASW Podium Spike to Center Spike";
                            default -> "";
                        }
                    );
                    var spike2Shot = AllianceFlipUtil.apply(spike1ToSpike2.getPoint(spike1ToSpike2.numPoints() - 1).position);
                    commands.add(
                        AutoCommons.shootWhenReady(spike2Shot, drive, shooter, pivot, kicker)
                        // .raceWith(
                        //     Commands.waitSeconds(2.5)
                        //     .andThen(
                        //         Commands.waitUntil(() -> 
                        //             !(
                        //                 intake.getIntakeCommand().equals(Optional.of(IntakeCommand.FEED_TO_KICKER)) || 
                        //                 kicker.hasNote()
                        //             )
                        //         )
                        //     )
                        // )
                        .deadlineWith(
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAim(spike2Shot, shooter, kicker, pivot),
                            AutoCommons.followPathFlipped(spike1ToSpike2, drive.translationSubsystem),
                            drive.rotationalSubsystem.pidControlledHeading(() -> Optional.of(AllianceFlipUtil.apply(Rotation2d.fromDegrees(135))))
                            .until(intake::hasNote)
                            .andThen(
                                AutoCommons.autoAim(spike2Shot, drive.rotationalSubsystem)
                            )
                        )
                    );
                }

                if(noteCount.asInt >= 4) {
                    var centerSpikeToPodiumSpike = AutoPaths.loadPath(
                        switch(startPosition) {
                            case Amp, SubwooferAmp -> "MASW Center Spike to Podium Spike";
                            case Podium, SubwooferSource -> "MASW Center Spike to Amp Spike";
                            default -> "";
                        }
                    );
                    var podiumSpikeShot = AllianceFlipUtil.apply(centerSpikeToPodiumSpike.getPoint(centerSpikeToPodiumSpike.numPoints() - 1).position);
                    commands.add(
                        AutoCommons.shootWhenReady(podiumSpikeShot, drive, shooter, pivot, kicker)
                        // .raceWith(
                        //     Commands.waitSeconds(2.5)
                        //     .andThen(
                        //         Commands.waitUntil(() -> 
                        //             !(
                        //                 intake.getIntakeCommand().equals(Optional.of(IntakeCommand.FEED_TO_KICKER)) || 
                        //                 kicker.hasNote()
                        //             )
                        //         )
                        //     )
                        // )
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
                    );
                }
                
                if(noteCount.asInt >= 5) {
                    var podiumSpikeToCenter = AutoPaths.loadPath("MASW Podium Spike to Center");
                    var centerToWing = AutoPaths.loadPath("R6N Center to Amp Wing");
                    var centerShot = AllianceFlipUtil.apply(centerToWing.getPoint(centerToWing.numPoints() - 1).position);
                    commands.add(
                        AutoCommons.shootWhenReady(centerShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            AutoCommons.autoAim(centerShot, shooter, kicker, pivot),
                            Commands.runOnce(noteVision::clearMemory)
                            .andThen(
                                AutoCommons.followPathFlipped(podiumSpikeToCenter, drive)
                                .onlyWhile(() -> !noteVision.hasTarget())
                                .andThen(
                                    intake.intake(drive::getChassisSpeeds)
                                    .deadlineWith(
                                        noteVision.autoIntake(() -> 1.5, drive, intake)
                                    ),
                                    AutoCommons.autoAim(centerShot, drive.rotationalSubsystem)
                                    .alongWith(
                                        AutoCommons.followPathFlipped(centerToWing, drive.translationSubsystem)
                                    )
                                )
                            )
                        )
                    );
                }

                return AutoCommons.setOdometryFlipped(startPosition.startPose, drive).andThen(commands.toArray(Command[]::new));
            }
        );
    }
}
