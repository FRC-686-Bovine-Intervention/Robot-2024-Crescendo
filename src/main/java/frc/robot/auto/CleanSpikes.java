package frc.robot.auto;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.SuperCommands;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.note.NoteVision;
import frc.robot.util.AllianceFlipUtil;

public class CleanSpikes extends AutoRoutine {
    public CleanSpikes(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public CleanSpikes(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super("Clean Spikes",
            List.of(),
            () -> {
                PathPlannerPath startToSpike = AutoPaths.loadPath(String.format(AutoPaths.startToSpike, "Amp"));
                PathPlannerPath ampSpikeToCenterSpike = AutoPaths.loadPath("Amp Wing to Center Spike");
                PathPlannerPath centerSpikeToPodiumSpike = AutoPaths.loadPath("Center Spike to Podium Spike");

                var preloadShot = startToSpike.getPoint(startToSpike.numPoints() - 1).position;
                var note1Shot = preloadShot;
                var note2Shot = ampSpikeToCenterSpike.getPoint(startToSpike.numPoints() - 1).position;
                var note3Shot = centerSpikeToPodiumSpike.getPoint(startToSpike.numPoints() - 1).position;

                return AutoCommons.setOdometryFlipped(AutoCommons.StartPosition.Amp.startPose, drive)
                    .andThen(
                        AutoCommons.followPathFlipped(startToSpike, drive.translationSubsystem)
                        .alongWith(
                            AutoCommons.autoAimAsIfAt(preloadShot, drive.rotationalSubsystem, shooter, pivot, kicker)
                            .deadlineWith(
                                Commands.waitUntil(() -> drive.getPose().getTranslation().getDistance(preloadShot) < 0.2)
                                .andThen(
                                    SuperCommands.shootWhenReady(shooter, pivot, kicker)
                                )
                            )
                        ),
                        intake.intake(drive::getChassisSpeeds)
                        .deadlineWith(
                            drive.translationSubsystem.fieldRelative(() -> AllianceFlipUtil.applyFieldRelative(new ChassisSpeeds(0.5, 0, 0)))
                        ),
                        AutoCommons.autoAimAsIfAt(note1Shot, drive.rotationalSubsystem, shooter, pivot, kicker)
                        .deadlineWith(
                            Commands.waitUntil(() -> drive.getPose().getTranslation().getDistance(note1Shot) < 0.2)
                            .andThen(
                                SuperCommands.shootWhenReady(shooter, pivot, kicker)
                            )
                        ),
                        AutoCommons.followPathFlipped(ampSpikeToCenterSpike, drive.translationSubsystem)
                        .alongWith(
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAimAsIfAt(note2Shot, drive.rotationalSubsystem, shooter, pivot, kicker)
                            .deadlineWith(
                                Commands.waitUntil(() -> drive.getPose().getTranslation().getDistance(note2Shot) < 0.2)
                                .andThen(
                                    SuperCommands.shootWhenReady(shooter, pivot, kicker)
                                )
                            )
                        ),
                        AutoCommons.followPathFlipped(centerSpikeToPodiumSpike, drive.translationSubsystem)
                        .alongWith(
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAimAsIfAt(note3Shot, drive.rotationalSubsystem, shooter, pivot, kicker)
                            .deadlineWith(
                                Commands.waitUntil(() -> drive.getPose().getTranslation().getDistance(note3Shot) < 0.2)
                                .andThen(
                                    SuperCommands.shootWhenReady(shooter, pivot, kicker)
                                )
                            )
                        )
                    )
                ;
            }
        );
    }
}