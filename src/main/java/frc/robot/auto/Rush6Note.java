package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.CenterNote;
import frc.robot.auto.AutoCommons.Count;
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
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> new StartPosition[]{
        StartPosition.Amp,
        StartPosition.SubwooferAmp,
    });

    private static final AutoQuestion<Count> noteCount = new AutoQuestion<>("Note Count", () -> new Count[]{
        Count.k5,
        Count.k4,
        Count.k3,
        Count.k2,
        Count.k1,
    });

    private static final AutoQuestion<CenterNote> firstCenterNote = new AutoQuestion<>("First Center Note", () -> new CenterNote[]{
        CenterNote.Note1,
        CenterNote.Note2,
    });

    private static final AutoQuestion<CenterNote> secondCenterNote = new AutoQuestion<>("Second Center Note", () -> new CenterNote[]{
        CenterNote.Note2,
        CenterNote.Note1,
    });

    public Rush6Note(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public Rush6Note(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super(
            "Rush 6 Note",
            List.of(
                startPosition,
                noteCount,
                firstCenterNote,
                secondCenterNote
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
        var startPosition = Rush6Note.startPosition.getResponse();
        var noteCount = Rush6Note.noteCount.getResponse();
        var firstCenterNote = Rush6Note.firstCenterNote.getResponse();
        var secondCenterNote = Rush6Note.secondCenterNote.getResponse();

        var commands = new ArrayList<Command>();

        if(noteCount.asInt >= 1) {
            var preloadShot = AllianceFlipUtil.apply(startPosition.startPose.getTranslation());
            commands.add(
                AutoCommons.shootWhenReady(preloadShot, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    AutoCommons.autoAim(preloadShot, shooter, kicker, pivot, drive.rotationalSubsystem)
                )
            );
        }

        if(noteCount.asInt >= 2) {
            var startToSpike = AutoPaths.loadPath("R6N Amp Start to Spike");
            var spikeShot = AllianceFlipUtil.apply(startToSpike.getPoint(startToSpike.numPoints() - 1).position);
            commands.add(
                AutoCommons.shootWhenReady(spikeShot, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    intake.intake(drive::getChassisSpeeds),
                    AutoCommons.autoAim(spikeShot, shooter, kicker, pivot, drive.rotationalSubsystem),
                    AutoCommons.followPathFlipped(startToSpike, drive.translationSubsystem)
                )
            );
        }

        if(noteCount.asInt >= 3) {
            var spikeToCenter = AutoPaths.loadPath("R6N Amp Spike to Center " + firstCenterNote.name());
            var centerToAmpWing = AutoPaths.loadPath("R6N Center to Amp Wing");
            var centerShot1 = AllianceFlipUtil.apply(centerToAmpWing.getPoint(centerToAmpWing.numPoints() - 1).position);
            commands.add(
                AutoCommons.shootWhenReady(centerShot1, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    AutoCommons.autoAim(centerShot1, shooter, kicker, pivot),
                    Commands.runOnce(noteVision::clearMemory)
                    .andThen(
                        AutoCommons.followPathFlipped(spikeToCenter, drive)
                        .onlyWhile(() -> !noteVision.hasTarget())
                        .andThen(
                            intake.intake(drive::getChassisSpeeds)
                            .deadlineWith(
                                noteVision.autoIntake(() -> 2, drive, intake)
                            ),
                            AutoCommons.autoAim(centerShot1, drive.rotationalSubsystem)
                            .alongWith(
                                AutoCommons.followPathFlipped(centerToAmpWing, drive.translationSubsystem)
                            )
                        )
                    )
                )
            );
        }

        if(noteCount.asInt >= 4) {
            var wingToCenter = AutoPaths.loadPath("R6N Amp Wing to Center " + secondCenterNote.name());
            var centerToAmpWing = AutoPaths.loadPath("R6N Center to Amp Wing");
            var centerShot2 = AllianceFlipUtil.apply(centerToAmpWing.getPoint(centerToAmpWing.numPoints() - 1).position);
            commands.add(
                AutoCommons.shootWhenReady(centerShot2, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    AutoCommons.autoAim(centerShot2, shooter, kicker, pivot),
                    Commands.runOnce(noteVision::clearMemory)
                    .andThen(
                        AutoCommons.followPathFlipped(wingToCenter, drive)
                        .onlyWhile(() -> !noteVision.hasTarget())
                        .andThen(
                            intake.intake(drive::getChassisSpeeds)
                            .deadlineWith(
                                noteVision.autoIntake(() -> 2, drive, intake)
                            ),
                            AutoCommons.autoAim(centerShot2, drive.rotationalSubsystem)
                            .alongWith(
                                AutoCommons.followPathFlipped(centerToAmpWing, drive.translationSubsystem)
                            )
                        )
                    )
                )
            );
        }

        if(noteCount.asInt >= 5) {
            var wingToCenter = AutoPaths.loadPath("R6N Amp Wing to Sneaky Stage");
            var centerToAmpWing = AutoPaths.loadPath("Center to Sneaky Stage");
            var centerShot3 = AllianceFlipUtil.apply(centerToAmpWing.getPoint(centerToAmpWing.numPoints() - 1).position);
            commands.add(
                AutoCommons.shootWhenReady(centerShot3, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    AutoCommons.autoAim(centerShot3, shooter, kicker),
                    Commands.runOnce(noteVision::clearMemory)
                    .andThen(
                        AutoCommons.followPathFlipped(wingToCenter, drive)
                        .onlyWhile(() -> !noteVision.hasTarget())
                        .andThen(
                            intake.intake(drive::getChassisSpeeds)
                            .deadlineWith(
                                noteVision.autoIntake(() -> 2, drive, intake)
                            ),
                            AutoCommons.autoAim(centerShot3, drive.rotationalSubsystem)
                            .alongWith(
                                AutoCommons.followPathFlipped(centerToAmpWing, drive.translationSubsystem)
                                .andThen(
                                    AutoCommons.autoAim(centerShot3, pivot)
                                )
                            )
                        )
                    )
                )
            );
        }
        
        return AutoCommons.setOdometryFlipped(startPosition.startPose, drive).andThen(commands.toArray(Command[]::new));
    }
}
