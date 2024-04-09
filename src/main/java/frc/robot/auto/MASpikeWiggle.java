package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
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

public class MASpikeWiggle extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> new StartPosition[]{
        StartPosition.Podium,
        StartPosition.Amp,
    });

    private static final AutoQuestion<Count> noteCount = new AutoQuestion<>("Note Count", () -> new Count[]{
        Count.k6,
        Count.k5,
        Count.k4,
        Count.k3,
        Count.k2,
        Count.k1,
    });

    private static final AutoQuestion<CenterNote> firstCenterNote = new AutoQuestion<>("First Center Note", () -> 
        noteCount.getResponse().asInt >= 5 ?
        new CenterNote[] {
            CenterNote.Note1,
            CenterNote.Note2,
            CenterNote.Note3,
        } :
        new CenterNote[]{}
    );

    private static final AutoQuestion<CenterNote> secondCenterNote = new AutoQuestion<>("Second Center Note", () -> 
        noteCount.getResponse().asInt >= 6 ?
        new CenterNote[] {
            CenterNote.Note2,
            CenterNote.Note1,
            CenterNote.Note3,
        } :
        new CenterNote[]{}
    );

    public MASpikeWiggle(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public MASpikeWiggle(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
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

        if(noteCount.asInt >= 1) {
            var preloadShot = AllianceFlipUtil.apply(startPosition.startPose.getTranslation());
            commands.add(
                AutoCommons.shootWhenReady(preloadShot, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    AutoCommons.autoAim(preloadShot, shooter, pivot, drive.rotationalSubsystem)
                )
            );
        }

        if(noteCount.asInt >= 2) {
            var startToSpike1 = AutoPaths.loadPath(
                switch(startPosition) {
                    case Amp, SubwooferAmp -> "Amp Start to Spike";
                    default -> "Podium Start to Spike";
                }
            );
            commands.add(
                AutoCommons.spikeNote(startToSpike1, drive, shooter, pivot, kicker, intake)
            );
        }

        if(noteCount.asInt >= 3) {
            var spike1ToSpike2 = AutoPaths.loadPath(
                switch(startPosition) {
                    case Amp, SubwooferAmp -> "MASW Amp Spike to Center Spike";
                    default -> "MASW Podium Spike to Center Spike";
                }
            );
            commands.add(
                AutoCommons.spikeNote(spike1ToSpike2, wiggleAngle, drive, shooter, pivot, kicker, intake)
            );
        }

        if(noteCount.asInt >= 4) {
            var spike2ToSpike3 = AutoPaths.loadPath(
                switch(startPosition) {
                    case Amp, SubwooferAmp -> "MASW Center Spike to Podium Spike";
                    default -> "MASW Center Spike to Amp Spike";
                }
            );
            commands.add(
                AutoCommons.spikeNote(spike2ToSpike3, wiggleAngle, drive, shooter, pivot, kicker, intake)
            );
        }
        
        if(noteCount.asInt >= 5) {
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
                    AutoCommons.centerNote(spikeToCenter, centerNote3ToAmpWing, centerNote1ToAmpWing, drive, shooter, pivot, kicker, intake, noteVision)
                ) : (
                    AutoCommons.centerNote(spikeToCenter, centerNote1ToAmpWing, centerNote3ToAmpWing, drive, shooter, pivot, kicker, intake, noteVision)
                )
            );
        }

        if(noteCount.asInt >= 6) {
            var wingToCenter = AutoPaths.loadPath("R6N Amp Wing to Center " + secondCenterNote.name());
            var centerNote1ToAmpWing = AutoPaths.loadPath("R6N Center Note1 to Amp Wing");
            var centerNote3ToAmpWing = AutoPaths.loadPath("S4N Center Note3 to Amp Wing");
            commands.add(
                secondCenterNote.equals(CenterNote.Note3) ? (
                    AutoCommons.centerNote(wingToCenter, centerNote3ToAmpWing, centerNote1ToAmpWing, drive, shooter, pivot, kicker, intake, noteVision)
                ) : (
                    AutoCommons.centerNote(wingToCenter, centerNote1ToAmpWing, centerNote3ToAmpWing, drive, shooter, pivot, kicker, intake, noteVision)
                )
            );
        }

        return AutoCommons.setOdometryFlipped(startPosition.startPose, drive).andThen(commands.toArray(Command[]::new));
    }
}
