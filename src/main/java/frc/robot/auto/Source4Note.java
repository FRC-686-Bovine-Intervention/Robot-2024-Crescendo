package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.CenterNote;
import frc.robot.auto.AutoCommons.StartPosition;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoQuestion.Settings;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.note.NoteVision;

public class Source4Note extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> {
        var podium = StartPosition.Podium.toEntry();
        var source = StartPosition.Source.toEntry();

        return Settings.from(podium, podium, source);
    });

    private static final AutoQuestion<Integer> noteCount = new AutoQuestion<>("Note Count", () -> {
        var k1 = Map.entry("1", 1);
        var k2 = Map.entry("2", 2);
        var k3 = Map.entry("3", 3);
        var k4 = Map.entry("4", 4);
        var k5 = Map.entry("5", 5);

        return Settings.from(k5, k5,k4,k3,k2,k1);
    });

    private static final AutoQuestion<Boolean> skipPreload = new AutoQuestion<>("Skip Preload", () -> {
        var no = Map.entry("No", false);
        var yes = Map.entry("Yes", true);

        return Settings.from(no, no, yes);
    });

    private static final AutoQuestion<CenterNote> firstCenterNote = new AutoQuestion<>("First Center Note", () -> {
        var c3 = CenterNote.Note3.toEntry();
        var c4 = CenterNote.Note4.toEntry();
        var c5 = CenterNote.Note5.toEntry();

        return (noteCount.getResponse() >= switch(startPosition.getResponse()) {
            case Podium -> 3;
            case Source -> 2;
            default -> 5;
        }) ? (
            Settings.from(c5, c3,c4,c5)
        ) : (
            Settings.empty()
        );
    });

    private static final AutoQuestion<CenterNote> secondCenterNote = new AutoQuestion<>("Second Center Note", () -> {
        var c3 = CenterNote.Note3.toEntry();
        var c4 = CenterNote.Note4.toEntry();
        var c5 = CenterNote.Note5.toEntry();

        return (noteCount.getResponse() >= switch(startPosition.getResponse()) {
            case Podium -> 4;
            case Source -> 3;
            default -> 5;
        }) ? (
            Settings.from(
                switch(firstCenterNote.getResponse()) {
                    default -> c3;
                    case Note5 -> c4;
                },
                c3,c4,c5
            )
        ) : (
            Settings.empty()
        );
    });

    private static final AutoQuestion<CenterNote> thirdCenterNote = new AutoQuestion<>("Second Center Note", () -> {
        var c3 = CenterNote.Note3.toEntry();
        var c4 = CenterNote.Note4.toEntry();
        var c5 = CenterNote.Note5.toEntry();

        return (noteCount.getResponse() >= switch(startPosition.getResponse()) {
            case Podium -> 5;
            case Source -> 4;
            default -> 5;
        }) ? (
            Settings.from(
                c3,
                c3,c4,c5
            )
        ) : (
            Settings.empty()
        );
    });

    public Source4Note(RobotContainer robot) {
        super(
            "Source 4 Note",
            List.of(
                startPosition,
                noteCount,
                skipPreload,
                firstCenterNote,
                secondCenterNote,
                thirdCenterNote
            )
        );
        this.drive = robot.drive;
        this.shooter = robot.shooter;
        this.pivot = robot.pivot;
        this.rollers = robot.rollers;
        this.noteVision = robot.noteVision;
    }

    private final Drive drive;
    private final Shooter shooter;
    private final Pivot pivot;
    private final Rollers rollers;
    private final NoteVision noteVision;

    @Override
    public Command generateCommand() {
        var startPosition = Source4Note.startPosition.getResponse();
        var noteCount = Source4Note.noteCount.getResponse();
        var skipPreload = Source4Note.skipPreload.getResponse();
        var firstCenterNote = Source4Note.firstCenterNote.getResponse();
        var secondCenterNote = Source4Note.secondCenterNote.getResponse();
        var thirdCenterNote = Source4Note.thirdCenterNote.getResponse();

        var commands = new ArrayList<Command>();

        if(noteCount >= 1 && !skipPreload) {
            commands.add(
                AutoCommons.preload(startPosition.startPose.getOurs().getTranslation(), drive, shooter, pivot, rollers)
            );
        }

        switch(startPosition) {
            default:
            case Podium:
                if(noteCount >= 2) {
                    var startToSpike = AutoPaths.loadPath("Podium Start to Spike");
                    commands.add(
                        AutoCommons.spikeNote(startToSpike, drive, shooter, pivot, rollers)
                    );
                }

                if(noteCount >= 3) {
                    var spikeToCenter = switch(firstCenterNote) {
                        default -> AutoPaths.loadPath("S4N Podium Spike to Center " + firstCenterNote.name());
                        case Note3 -> AutoPaths.loadPath("MASW Podium Spike to Center Note3");
                    };
                    var centerNote5ToSourceWing = AutoPaths.loadPath("S4N Center Note5 to Source Wing");
                    var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
                    commands.add(
                        firstCenterNote.equals(CenterNote.Note3) ? (
                            AutoCommons.centerNote(spikeToCenter, centerNote3ToAmpWing, centerNote5ToSourceWing, drive, shooter, pivot, rollers, noteVision)
                        ) : (
                            AutoCommons.centerNote(spikeToCenter, centerNote5ToSourceWing, centerNote3ToAmpWing, drive, shooter, pivot, rollers, noteVision)
                        )
                    );
                }

                if(noteCount >= 4) {
                    var sourceWingToCenter = AutoPaths.loadPath("S4N Source Wing to Center " + secondCenterNote.name());
                    var ampWingToCenter = AutoPaths.loadPath("R6N Amp Wing to Center Note3");
                    var centerNote5ToSourceWing = AutoPaths.loadPath("S4N Center Note5 to Source Wing");
                    var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
                    var toAmpWingStartPoint = AutoCommons.getFirstPoint(centerNote3ToAmpWing);
                    var toSourceWingStartPoint = AutoCommons.getFirstPoint(centerNote5ToSourceWing);
                    BooleanSupplier isAmpWing = () -> drive.getPose().getTranslation().nearest(List.of(toAmpWingStartPoint, toSourceWingStartPoint)).equals(toAmpWingStartPoint);
                    commands.add(
                        Commands.either(
                            AutoCommons.centerNote(ampWingToCenter, centerNote3ToAmpWing, centerNote5ToSourceWing, drive, shooter, pivot, rollers, noteVision), 
                            secondCenterNote.equals(CenterNote.Note3) ? (
                                AutoCommons.centerNote(sourceWingToCenter, centerNote3ToAmpWing, centerNote5ToSourceWing, drive, shooter, pivot, rollers, noteVision)
                            ) : (
                                AutoCommons.centerNote(sourceWingToCenter, centerNote5ToSourceWing, centerNote3ToAmpWing, drive, shooter, pivot, rollers, noteVision)
                            ), 
                            isAmpWing
                        )
                    );
                }
            break;
            case Source:
                if(noteCount >= 2) {
                    var wingToCenter = AutoPaths.loadPath("S4N Source Wing to Center " + firstCenterNote.name());
                    var centerNote5ToSourceWing = AutoPaths.loadPath("S4N Center Note5 to Source Wing");
                    var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
                    commands.add(
                        firstCenterNote.equals(CenterNote.Note3) ? (
                            AutoCommons.centerNote(wingToCenter, centerNote3ToAmpWing, centerNote5ToSourceWing, drive, shooter, pivot, rollers, noteVision)
                        ) : (
                            AutoCommons.centerNote(wingToCenter, centerNote5ToSourceWing, centerNote3ToAmpWing, drive, shooter, pivot, rollers, noteVision)
                        )
                    );
                }

                if(noteCount >= 3) {
                    var sourceWingToCenter = AutoPaths.loadPath("S4N Source Wing to Center " + secondCenterNote.name());
                    var ampWingToCenter = AutoPaths.loadPath("R6N Amp Wing to Center Note3");
                    var centerNote5ToSourceWing = AutoPaths.loadPath("S4N Center Note5 to Source Wing");
                    var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
                    var toAmpWingStartPoint = AutoCommons.getFirstPoint(centerNote3ToAmpWing);
                    var toSourceWingStartPoint = AutoCommons.getFirstPoint(centerNote5ToSourceWing);
                    BooleanSupplier isAmpWing = () -> drive.getPose().getTranslation().nearest(List.of(toAmpWingStartPoint, toSourceWingStartPoint)).equals(toAmpWingStartPoint);
                    commands.add(
                        Commands.either(
                            AutoCommons.centerNote(ampWingToCenter, centerNote3ToAmpWing, centerNote5ToSourceWing, drive, shooter, pivot, rollers, noteVision), 
                            secondCenterNote.equals(CenterNote.Note3) ? (
                                AutoCommons.centerNote(sourceWingToCenter, centerNote3ToAmpWing, centerNote5ToSourceWing, drive, shooter, pivot, rollers, noteVision)
                            ) : (
                                AutoCommons.centerNote(sourceWingToCenter, centerNote5ToSourceWing, centerNote3ToAmpWing, drive, shooter, pivot, rollers, noteVision)
                            ), 
                            isAmpWing
                        )
                    );
                }

                if(noteCount >= 4) {
                    var sourceWingToCenter = AutoPaths.loadPath("S4N Source Wing to Center " + thirdCenterNote.name());
                    var ampWingToCenter = AutoPaths.loadPath("R6N Amp Wing to Center Note3");
                    var centerNote5ToSourceWing = AutoPaths.loadPath("S4N Center Note5 to Source Wing");
                    var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
                    var toAmpWingStartPoint = AutoCommons.getFirstPoint(centerNote3ToAmpWing);
                    var toSourceWingStartPoint = AutoCommons.getFirstPoint(centerNote5ToSourceWing);
                    BooleanSupplier isAmpWing = () -> drive.getPose().getTranslation().nearest(List.of(toAmpWingStartPoint, toSourceWingStartPoint)).equals(toAmpWingStartPoint);
                    commands.add(
                        Commands.either(
                            AutoCommons.centerNote(ampWingToCenter, centerNote3ToAmpWing, centerNote5ToSourceWing, drive, shooter, pivot, rollers, noteVision), 
                            thirdCenterNote.equals(CenterNote.Note3) ? (
                                AutoCommons.centerNote(sourceWingToCenter, centerNote3ToAmpWing, centerNote5ToSourceWing, drive, shooter, pivot, rollers, noteVision)
                            ) : (
                                AutoCommons.centerNote(sourceWingToCenter, centerNote5ToSourceWing, centerNote3ToAmpWing, drive, shooter, pivot, rollers, noteVision)
                            ), 
                            isAmpWing
                        )
                    );
                }
            break;
        }
        
        return AutoCommons.setOdometryFlipped(startPosition.startPose, drive).andThen(commands.toArray(Command[]::new));
    }
}
