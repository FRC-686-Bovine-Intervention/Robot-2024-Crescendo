package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.StartPosition;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoQuestion.Settings;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.note.NoteVision;
import frc.robot.util.AllianceFlipUtil;

public class Disruptor extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> {
        var source = StartPosition.Source.toEntry();

        return Settings.from(source, source);
    });

    public Disruptor(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.rollers, robot.noteVision);
    }
    public Disruptor(Drive drive, Shooter shooter, Pivot pivot, Rollers rollers, NoteVision noteVision) {
        super(
            "Disruptor",
            List.of(
                startPosition
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
        var disruptorPath = AutoPaths.loadPath("Disruptor");
        var centerToWing = AutoPaths.loadPath("R6N Center Note1 to Amp Wing");

        var preloadShot = AllianceFlipUtil.apply(startPosition.getResponse().startPose.getTranslation());
        var centerShot = AllianceFlipUtil.apply(centerToWing.getPoint(centerToWing.numPoints() - 1).position);

        return AutoCommons.setOdometryFlipped(startPosition.getResponse().startPose, drive)
            .andThen(
                AutoCommons.shootWhenReady(10, drive, shooter, pivot, rollers)
                .deadlineWith(
                    AutoCommons.autoAim(preloadShot, shooter, pivot, drive.rotationalSubsystem)
                ),
                AutoCommons.shootWhenReady(10, drive, shooter, pivot, rollers)
                .deadlineWith(
                    AutoCommons.autoAim(centerShot, shooter, pivot),
                    AutoCommons.followPathFlipped(disruptorPath, drive)
                    .andThen(
                        Commands.runOnce(noteVision::clearMemory),
                        rollers.setIntakeGoalCommand(Intake.Goal.INTAKE)
                        .deadlineWith(
                            noteVision.autoIntake(() -> 2, rollers::noNote, drive)
                        ),
                        AutoCommons.autoAim(centerShot, drive.rotationalSubsystem)
                        .alongWith(
                            AutoCommons.followPathFlipped(centerToWing, drive.translationSubsystem)
                        )
                    )
                )
            )
        ;
    }
}