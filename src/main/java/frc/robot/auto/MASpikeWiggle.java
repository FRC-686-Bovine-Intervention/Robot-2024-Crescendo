package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
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

public class MASpikeWiggle extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> {
        var podium = StartPosition.Podium.toEntry();
        var amp = StartPosition.Amp.toEntry();

        return Settings.from(podium.getValue(), podium, amp);
    });

    private static final AutoQuestion<Integer> noteCount = new AutoQuestion<Integer>("Note Count", () -> {
        var k1 = Map.entry("1", 1);
        var k2 = Map.entry("2", 2);
        var k3 = Map.entry("3", 3);
        var k4 = Map.entry("4", 4);
        var k5 = Map.entry("5", 5);
        var k6 = Map.entry("6", 6);

        return Settings.from(k6.getValue(), k6,k5,k4,k3,k2,k1);
    });

    private static final AutoQuestion<CenterNote> firstCenterNote = new AutoQuestion<>("First Center Note", () -> {
        var c1 = CenterNote.Note1.toEntry();
        var c2 = CenterNote.Note2.toEntry();
        var c3 = CenterNote.Note3.toEntry();

        return (noteCount.getResponse() >= 5) ? (
            Settings.from(c1.getValue(), c1,c2,c3)
        ) : (
            Settings.empty()
        );
    });

    private static final AutoQuestion<CenterNote> secondCenterNote = new AutoQuestion<>("Second Center Note", () -> {
        var c1 = CenterNote.Note1.toEntry();
        var c2 = CenterNote.Note2.toEntry();
        var c3 = CenterNote.Note3.toEntry();

        return (noteCount.getResponse() >= 6) ? (
            Settings.from(
                switch(firstCenterNote.getResponse()) {
                    default -> c2.getValue();
                    case Note2 -> c3.getValue();
                },
                c1,c2,c3
            )
        ) : (
            Settings.empty()
        );
    });

    public MASpikeWiggle(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.rollers, robot.noteVision);
    }
    public MASpikeWiggle(Drive drive, Shooter shooter, Pivot pivot, Rollers rollers, NoteVision noteVision) {
        super(
            "MA Spike Wiggle",
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
        var startPosition = MASpikeWiggle.startPosition.getResponse();
        var noteCount = MASpikeWiggle.noteCount.getResponse();
        var firstCenterNote = MASpikeWiggle.firstCenterNote.getResponse();
        var secondCenterNote = MASpikeWiggle.secondCenterNote.getResponse();

        var wiggleAngle = Rotation2d.fromDegrees(
            switch(startPosition) {
                case Amp, SubwooferAmp -> 135;
                default -> -135;
            }
        );
        
        var commands = new ArrayList<Command>();

        if(noteCount >= 1) {
            commands.add(
                AutoCommons.preload(startPosition.startPose.getTranslation(), drive, shooter, pivot, rollers)
            );
        }

        if(noteCount >= 2) {
            var startToSpike1 = AutoPaths.loadPath(
                switch(startPosition) {
                    case Amp, SubwooferAmp -> "Amp Start to Spike";
                    default -> "Podium Start to Spike";
                }
            );
            commands.add(
                AutoCommons.spikeNote(startToSpike1, drive, shooter, pivot, rollers)
            );
        }

        if(noteCount >= 3) {
            var spike1ToSpike2 = AutoPaths.loadPath(
                switch(startPosition) {
                    case Amp, SubwooferAmp -> "MASW Amp Spike to Center Spike";
                    default -> "MASW Podium Spike to Center Spike";
                }
            );
            commands.add(
                AutoCommons.spikeNote(spike1ToSpike2, wiggleAngle, drive, shooter, pivot, rollers)
            );
        }

        if(noteCount >= 4) {
            var spike2ToSpike3 = AutoPaths.loadPath(
                switch(startPosition) {
                    case Amp, SubwooferAmp -> "MASW Center Spike to Podium Spike";
                    default -> "MASW Center Spike to Amp Spike";
                }
            );
            commands.add(
                AutoCommons.spikeNote(spike2ToSpike3, wiggleAngle, drive, shooter, pivot, rollers)
            );
        }
        
        if(noteCount >= 5) {
            var spikeToCenter = AutoPaths.loadPath(
                switch(startPosition) {
                    case Amp, SubwooferAmp -> "MASW Podium Spike to Center " + firstCenterNote.name();
                    default -> "R6N Amp Spike to Center " + firstCenterNote.name();
                }
            );
            var centerNote1ToAmpWing = AutoPaths.loadPath("R6N Center Note1 to Amp Wing");
            var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
            commands.add(
                firstCenterNote.equals(CenterNote.Note3) ? (
                    AutoCommons.centerNote(spikeToCenter, centerNote3ToAmpWing, centerNote1ToAmpWing, drive, shooter, pivot, rollers, noteVision)
                ) : (
                    AutoCommons.centerNote(spikeToCenter, centerNote1ToAmpWing, centerNote3ToAmpWing, drive, shooter, pivot, rollers, noteVision)
                )
            );
        }

        if(noteCount >= 6) {
            var wingToCenter = AutoPaths.loadPath("R6N Amp Wing to Center " + secondCenterNote.name());
            var centerNote1ToAmpWing = AutoPaths.loadPath("R6N Center Note1 to Amp Wing");
            var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
            commands.add(
                secondCenterNote.equals(CenterNote.Note3) ? (
                    AutoCommons.centerNote(wingToCenter, centerNote3ToAmpWing, centerNote1ToAmpWing, drive, shooter, pivot, rollers, noteVision)
                ) : (
                    AutoCommons.centerNote(wingToCenter, centerNote1ToAmpWing, centerNote3ToAmpWing, drive, shooter, pivot, rollers, noteVision)
                )
            );
        }

        return AutoCommons.setOdometryFlipped(startPosition.startPose, drive).andThen(commands.toArray(Command[]::new));
    }
}
