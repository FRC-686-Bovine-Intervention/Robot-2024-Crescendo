package frc.robot.auto;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
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

public class CleanSpikes extends AutoRoutine {
    public CleanSpikes(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public CleanSpikes(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super("Clean Spikes",
            List.of(),
            () -> {
                PathPlannerPath startToSpike = AutoPaths.loadPath(String.format(AutoPaths.startToSpike, "Amp"));
                PathPlannerPath ampSpikeToCenterSpike = AutoPaths.loadPath("Amp Spike to Center Spike");
                PathPlannerPath centerSpikeToPodiumSpike = AutoPaths.loadPath("Center Spike to Podium Spike");

                var preloadShot = StartPosition.Amp.startPose.getTranslation();
                var ampSpikeShot = startToSpike.getPoint(startToSpike.numPoints() - 1).position;
                var note2Shot = ampSpikeToCenterSpike.getPoint(startToSpike.numPoints() - 1).position;
                var note3Shot = centerSpikeToPodiumSpike.getPoint(startToSpike.numPoints() - 1).position;

                return AutoCommons.setOdometryFlipped(StartPosition.Amp.startPose, drive)
                    .andThen(
                        AutoCommons.shootWhenReady(preloadShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            AutoCommons.autoAim(preloadShot, shooter, pivot, drive.rotationalSubsystem)
                        ),
                        AutoCommons.shootWhenReady(ampSpikeShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            Commands.print("[Rush6Note] Shot Preload"),
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAim(ampSpikeShot, shooter, pivot, drive.rotationalSubsystem),
                            AutoCommons.followPathFlipped(startToSpike, drive.translationSubsystem)
                        ),
                        AutoCommons.shootWhenReady(note2Shot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAim(note2Shot, shooter, pivot, drive.rotationalSubsystem),
                            AutoCommons.followPathFlipped(ampSpikeToCenterSpike, drive.translationSubsystem)
                        ),
                        AutoCommons.shootWhenReady(note3Shot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAim(note3Shot, shooter, pivot, drive.rotationalSubsystem),
                            AutoCommons.followPathFlipped(centerSpikeToPodiumSpike, drive.translationSubsystem)
                        )
                    )
                ;
            }
        );
    }
}