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

public class Source4Note extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> new StartPosition[]{
        StartPosition.Podium,
        StartPosition.Source,
    });
    
    private static final AutoQuestion<Count> noteCount = new AutoQuestion<>("Note Count", () -> 
        switch(startPosition.getResponse()) {
            case Podium -> new Count[] {
                Count.k4,
                Count.k3,
                Count.k2,
                Count.k1,
            };
            case Source -> new Count[] {
                Count.k3,
                Count.k2,
                Count.k1,
            };
            default -> new Count[]{};
        }
    );

    private static final AutoQuestion<CenterNote> firstCenterNote = new AutoQuestion<>("First Center Note", () -> 
        noteCount.getResponse().asInt >= switch(startPosition.getResponse()) {
            case Podium -> 3;
            case Source -> 2;
            default -> 5;
        } ?
        new CenterNote[] {
            CenterNote.Note5,
            CenterNote.Note4,
            CenterNote.Note3,
        } :
        new CenterNote[]{}
    );

    private static final AutoQuestion<CenterNote> secondCenterNote = new AutoQuestion<>("Second Center Note", () -> 
        noteCount.getResponse().asInt >= switch(startPosition.getResponse()) {
            case Podium -> 4;
            case Source -> 3;
            default -> 5;
        } ?
        new CenterNote[] {
            CenterNote.Note5,
            CenterNote.Note4,
            CenterNote.Note3,
        } :
        new CenterNote[]{}
    );

    public Source4Note(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public Source4Note(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super(
            "Source 4 Note",
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
        var startPosition = Source4Note.startPosition.getResponse();
        var noteCount = Source4Note.noteCount.getResponse();
        var firstCenterNote = Source4Note.firstCenterNote.getResponse();
        var secondCenterNote = Source4Note.secondCenterNote.getResponse();

        var commands = new ArrayList<Command>();

        if(noteCount.asInt >= 1) {
            var preloadShot = AllianceFlipUtil.apply(startPosition.startPose.getTranslation());
            commands.add(
                AutoCommons.shootWhenReady(preloadShot, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    AutoCommons.autoAim(preloadShot, shooter, pivot, drive.rotationalSubsystem)
                )
            );
        }

        switch(startPosition) {
            default:
            case Podium:
                if(noteCount.asInt >= 2) {
                    var startToSpike = AutoPaths.loadPath("Podium Start to Spike");
                    var spikeShot = AllianceFlipUtil.apply(startToSpike.getPoint(startToSpike.numPoints() - 1).position);
                    commands.add(
                        AutoCommons.shootWhenReady(spikeShot, 10, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            intake.intake(drive::getChassisSpeeds),
                            AutoCommons.autoAim(spikeShot, shooter, pivot, drive.rotationalSubsystem),
                            AutoCommons.followPathFlipped(startToSpike, drive.translationSubsystem)
                        )
                    );
                }

                if(noteCount.asInt >= 3) {
                    var spikeToCenter = switch(firstCenterNote) {
                        default -> AutoPaths.loadPath("S4N Podium Spike to Center " + firstCenterNote.name());
                        case Note3 -> AutoPaths.loadPath("MASW Podium Spike to Center Note3");
                    };
                    var centerNote5ToSourceWing = AutoPaths.loadPath("S4N Center Note5 to Source Wing");
                    var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
                    commands.add(
                        firstCenterNote.equals(CenterNote.Note3) ? (
                            AutoCommons.centerNote(spikeToCenter, centerNote3ToAmpWing, centerNote5ToSourceWing, drive, shooter, pivot, kicker, intake, noteVision)
                        ) : (
                            AutoCommons.centerNote(spikeToCenter, centerNote5ToSourceWing, centerNote3ToAmpWing, drive, shooter, pivot, kicker, intake, noteVision)
                        )
                    );
                }

                if(noteCount.asInt >= 4) {
                    var wingToCenter = switch(firstCenterNote) {
                        case Note3 -> AutoPaths.loadPath("R6N Amp Wing to Center Note3");
                        default -> AutoPaths.loadPath("R6N Amp Wing to Center Note3");
                    };
                    var centerNote1ToAmpWing = AutoPaths.loadPath("R6N Center Note1 to Amp Wing");
                    var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
                    commands.add(
                        AutoCommons.centerNote(wingToCenter, centerNote1ToAmpWing, centerNote3ToAmpWing, drive, shooter, pivot, kicker, intake, noteVision)
                    );
                }
            break;
            case Source:
                if(noteCount.asInt >= 2) {
                    var wingToCenter = AutoPaths.loadPath("S4N Source Wing to Center " + firstCenterNote.name());
                    var centerNote5ToSourceWing = AutoPaths.loadPath("S4N Center Note5 to Source Wing");
                    var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
                    commands.add(
                        firstCenterNote.equals(CenterNote.Note3) ? (
                            AutoCommons.centerNote(wingToCenter, centerNote3ToAmpWing, centerNote5ToSourceWing, drive, shooter, pivot, kicker, intake, noteVision)
                        ) : (
                            AutoCommons.centerNote(wingToCenter, centerNote5ToSourceWing, centerNote3ToAmpWing, drive, shooter, pivot, kicker, intake, noteVision)
                        )
                    );
                }
            break;
        }

        if(noteCount.asInt >= 3) {
            var spikeToCenter = AutoPaths.loadPath("R6N Amp Spike to Center " + firstCenterNote.name());
            var centerNote1ToAmpWing = AutoPaths.loadPath("R6N Center Note1 to Amp Wing");
            var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
            commands.add(
                AutoCommons.centerNote(spikeToCenter, centerNote1ToAmpWing, centerNote3ToAmpWing, drive, shooter, pivot, kicker, intake, noteVision)
            );
        }

        if(noteCount.asInt >= 4) {
            var wingToCenter = AutoPaths.loadPath("R6N Amp Wing to Center " + secondCenterNote.name());
            var centerNote1ToAmpWing = AutoPaths.loadPath("R6N Center Note1 to Amp Wing");
            var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
            commands.add(
                AutoCommons.centerNote(wingToCenter, centerNote1ToAmpWing, centerNote3ToAmpWing, drive, shooter, pivot, kicker, intake, noteVision)
            );
        }
        
        return AutoCommons.setOdometryFlipped(startPosition.startPose, drive).andThen(commands.toArray(Command[]::new));
    }
}
