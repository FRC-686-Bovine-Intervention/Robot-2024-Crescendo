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

public class Disruptor extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> new StartPosition[]{StartPosition.Source});

    public Disruptor(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public Disruptor(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super(
            "Disruptor",
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
        var disruptorPath = AutoPaths.loadPath("Disruptor");
        var centerToWing = AutoPaths.loadPath("R6N Center Note1 to Amp Wing");

        var preloadShot = AllianceFlipUtil.apply(startPosition.getResponse().startPose.getTranslation());
        var centerShot = AllianceFlipUtil.apply(centerToWing.getPoint(centerToWing.numPoints() - 1).position);

        return AutoCommons.setOdometryFlipped(startPosition.getResponse().startPose, drive)
            .andThen(
                AutoCommons.shootWhenReady(preloadShot, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    AutoCommons.autoAim(preloadShot, shooter, pivot, drive.rotationalSubsystem)
                ),
                AutoCommons.shootWhenReady(centerShot, 10, drive, shooter, pivot, kicker)
                .deadlineWith(
                    AutoCommons.autoAim(centerShot, shooter, pivot),
                    AutoCommons.followPathFlipped(disruptorPath, drive)
                    .andThen(
                        Commands.runOnce(noteVision::clearMemory),
                        intake.intake(drive::getRobotRelativeSpeeds)
                        .deadlineWith(
                            noteVision.autoIntake(() -> 2, drive, intake)
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