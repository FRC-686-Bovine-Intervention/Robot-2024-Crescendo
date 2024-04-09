package frc.robot.auto;

import java.util.List;

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

public class SneakySource3Note extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> new StartPosition[] {
        StartPosition.Source
    });

    public SneakySource3Note(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public SneakySource3Note(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super(
            "Sneaky Source 3 Note",
            List.of(
                startPosition
            )
        );
        this.drive = drive;
        this.shooter = shooter;
        this.pivot = pivot;
        this.kicker = kicker;
        this.intake = intake;
        this.noteVision = noteVision;
    }

    private final Drive drive;
    private final Shooter shooter;
    private final Pivot pivot;
    private final Kicker kicker;
    private final Intake intake;
    private final NoteVision noteVision;

    @Override
    public Command generateCommand() {
        var wingToCenter = AutoPaths.loadPath("Source Wing to Center");
        var centerToWingStop = AutoPaths.loadPath("Center to Source Wing With Stop");
        var centerToSneakyStage = AutoPaths.loadPath("Center to Sneaky Stage");

        var preloadShot = AllianceFlipUtil.apply(startPosition.getResponse().startPose.getTranslation());
        var centerShot1 = AllianceFlipUtil.apply(centerToWingStop.getPoint(centerToWingStop.numPoints() - 1).position);
        var sneakyStageShot = AllianceFlipUtil.apply(centerToSneakyStage.getPoint(centerToSneakyStage.numPoints() - 1).position);

        return AutoCommons.setOdometryFlipped(StartPosition.Source.startPose, drive)
            .andThen(
                AutoCommons.shootWhenReady(preloadShot, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    AutoCommons.autoAim(preloadShot, shooter, pivot, drive.rotationalSubsystem)
                ),
                AutoCommons.shootWhenReady(centerShot1, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    Commands.print("[Source3Note] Shot Preload"),
                    AutoCommons.autoAim(centerShot1, shooter, pivot),
                    Commands.runOnce(noteVision::clearMemory)
                    .andThen(
                        AutoCommons.followPathFlipped(wingToCenter, drive)
                        .onlyWhile(() -> !noteVision.hasTarget())
                        .andThen(
                            intake.intake(drive::getRobotRelativeSpeeds)
                            .deadlineWith(
                                noteVision.autoIntake(() -> 2, drive, intake)
                            ),
                            AutoCommons.autoAim(centerShot1, drive.rotationalSubsystem)
                            .alongWith(
                                AutoCommons.followPathFlipped(centerToWingStop, drive.translationSubsystem)
                            )
                        )
                    )
                ),
                AutoCommons.shootWhenReady(sneakyStageShot, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    Commands.print("[Source3Note] Shot Center 1"),
                    AutoCommons.autoAim(sneakyStageShot, shooter),
                    Commands.runOnce(noteVision::clearMemory)
                    .andThen(
                        AutoCommons.followPathFlipped(wingToCenter, drive)
                        .onlyWhile(() -> !noteVision.hasTarget()),
                        intake.intake(drive::getRobotRelativeSpeeds)
                        .deadlineWith(
                            noteVision.autoIntake(() -> 2, drive, intake)
                        ),
                        AutoCommons.autoAim(sneakyStageShot, drive.rotationalSubsystem)
                        .alongWith(
                            AutoCommons.followPathFlipped(centerToSneakyStage, drive.translationSubsystem)
                            .andThen(
                                AutoCommons.autoAim(sneakyStageShot, pivot)
                            )
                        )
                    )
                ),
                Commands.print("[Source3Note] Shot Sneaky Stage")
            )
        ;
    }
}
