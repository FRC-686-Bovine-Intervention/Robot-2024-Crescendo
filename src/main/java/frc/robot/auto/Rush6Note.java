package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
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

public class Rush6Note extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> {
        var amp = StartPosition.Amp.toEntry();

        return Settings.from(amp, amp);
    });

    private static final AutoQuestion<Integer> noteCount = new AutoQuestion<>("Note Count", () -> {
        var k1 = Map.entry("1", 1);
        var k2 = Map.entry("2", 2);
        var k3 = Map.entry("3", 3);
        var k4 = Map.entry("4", 4);
        var k5 = Map.entry("5", 5);

        return Settings.from(k5, k5,k4,k3,k2,k1);
    });

    private static final AutoQuestion<CenterNote> firstCenterNote = new AutoQuestion<>("First Center Note", () -> {
        var c1 = CenterNote.Note1.toEntry();
        var c2 = CenterNote.Note2.toEntry();
        var c3 = CenterNote.Note3.toEntry();

        return (noteCount.getResponse() >= 3) ? (
            Settings.from(c1, c1,c2,c3)
        ) : (
            Settings.empty()
        );
    });

    private static final AutoQuestion<CenterNote> secondCenterNote = new AutoQuestion<>("Second Center Note", () -> {
        var c1 = CenterNote.Note1.toEntry();
        var c2 = CenterNote.Note2.toEntry();
        var c3 = CenterNote.Note3.toEntry();

        return (noteCount.getResponse() >= 4) ? (
            Settings.from(
                switch(firstCenterNote.getResponse()) {
                    default -> c3;
                    case Note1 -> c2;
                },
                c1,c2,c3
            )
        ) : (
            Settings.empty()
        );
    });

    private static final AutoQuestion<CenterNote> thirdCenterNote = new AutoQuestion<>("Second Center Note", () -> {
        var c1 = CenterNote.Note1.toEntry();
        var c2 = CenterNote.Note2.toEntry();
        var c3 = CenterNote.Note3.toEntry();

        return (noteCount.getResponse() >= 5) ? (
            Settings.from(
                c3,
                c1,c2,c3
            )
        ) : (
            Settings.empty()
        );
    });

    public Rush6Note(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.rollers, robot.noteVision);
    }
    public Rush6Note(Drive drive, Shooter shooter, Pivot pivot, Rollers rollers, NoteVision noteVision) {
        super(
            "Rush 6 Note",
            List.of(
                startPosition,
                noteCount,
                firstCenterNote,
                secondCenterNote,
                thirdCenterNote
            )
        );
        this.drive = drive;
        this.shooter = shooter;
        this.pivot = pivot;
        this.rollers = rollers;
        this.noteVision = noteVision;
    }

    private final Drive drive;
    private final Shooter shooter;
    private final Pivot pivot;
    private final Rollers rollers;
    private final NoteVision noteVision;

    @Override
    public Command generateCommand() {
        var startPosition = Rush6Note.startPosition.getResponse();
        var noteCount = Rush6Note.noteCount.getResponse();
        var firstCenterNote = Rush6Note.firstCenterNote.getResponse();
        var secondCenterNote = Rush6Note.secondCenterNote.getResponse();
        var thirdCenterNote = Rush6Note.thirdCenterNote.getResponse();

        var commands = new ArrayList<Command>();

        if(noteCount >= 1) {
            commands.add(
                AutoCommons.preload(startPosition.startPose.getTranslation(), drive, shooter, pivot, rollers)
            );
        }

        if(noteCount >= 2) {
            var startToSpike = AutoPaths.loadPath("R6N Amp Start to Spike");
            commands.add(
                AutoCommons.spikeNote(startToSpike, drive, shooter, pivot, rollers)
            );
        }

        if(noteCount >= 3) {
            var spikeToCenter = AutoPaths.loadPath("R6N Amp Spike to Center " + firstCenterNote.name());
            var centerNote1ToAmpWing = AutoPaths.loadPath("R6N Center Note1 to Amp Wing");
            var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
            commands.add(
                AutoCommons.centerNote(spikeToCenter, centerNote1ToAmpWing, centerNote3ToAmpWing, drive, shooter, pivot, rollers, noteVision)
            );
        }

        if(noteCount >= 4) {
            var wingToCenter = AutoPaths.loadPath("R6N Amp Wing to Center " + secondCenterNote.name());
            var centerNote1ToAmpWing = AutoPaths.loadPath("R6N Center Note1 to Amp Wing");
            var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
            commands.add(
                AutoCommons.centerNote(wingToCenter, centerNote1ToAmpWing, centerNote3ToAmpWing, drive, shooter, pivot, rollers, noteVision)
            );
        }

        if(noteCount >= 5) {
            var wingToCenter = AutoPaths.loadPath("R6N Amp Wing to Center " + thirdCenterNote.name());
            var centerNote1ToAmpWing = AutoPaths.loadPath("R6N Center Note1 to Amp Wing");
            var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
            commands.add(
                thirdCenterNote.equals(CenterNote.Note3) ? (
                    AutoCommons.centerNote(wingToCenter, centerNote3ToAmpWing, centerNote1ToAmpWing, drive, shooter, pivot, rollers, noteVision)
                ) : (
                    AutoCommons.centerNote(wingToCenter, centerNote1ToAmpWing, centerNote3ToAmpWing, drive, shooter, pivot, rollers, noteVision)
                )
            );
        }
        
        return AutoCommons.setOdometryFlipped(startPosition.startPose, drive).andThen(commands.toArray(Command[]::new));
    }
}
