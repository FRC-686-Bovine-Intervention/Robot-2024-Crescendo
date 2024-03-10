package frc.robot.auto;

import java.util.List;

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

public class Source4Note extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", () -> new StartPosition[]{StartPosition.Source});

    public Source4Note(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public Source4Note(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super("Source 4 Note",
            List.of(startPosition),
            () -> {
                var wingToCenter = AutoPaths.loadPath("Source Wing to Center");
                var centerToWingStop = AutoPaths.loadPath("Center to Source Wing With Stop");

                var preloadShot = AllianceFlipUtil.apply(StartPosition.Source.startPose.getTranslation());
                var centerShot1 = AllianceFlipUtil.apply(centerToWingStop.getPoint(centerToWingStop.numPoints() - 1).position);
                var centerShot2 = centerShot1;
                var centerShot3 = centerShot2;

                return AutoCommons.setOdometryFlipped(StartPosition.Source.startPose, drive)
                    .andThen(
                        AutoCommons.shootWhenReady(preloadShot, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            AutoCommons.autoAim(preloadShot, shooter, pivot, drive.rotationalSubsystem)
                        ),
                        AutoCommons.shootWhenReady(centerShot1, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            Commands.print("[Source4Note] Shot Preload"),
                            AutoCommons.autoAim(centerShot1, shooter, pivot),
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
                                        AutoCommons.followPathFlipped(centerToWingStop, drive.translationSubsystem)
                                    )
                                )
                            )
                        ),
                        AutoCommons.shootWhenReady(centerShot2, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            Commands.print("[Source4Note] Shot Center 1"),
                            AutoCommons.autoAim(centerShot2, shooter, pivot),
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
                                        AutoCommons.followPathFlipped(centerToWingStop, drive.translationSubsystem)
                                    )
                                )
                            )
                        ),
                        AutoCommons.shootWhenReady(centerShot3, drive, shooter, pivot, kicker)
                        .deadlineWith(
                            Commands.print("[Source4Note] Shot Center 1"),
                            AutoCommons.autoAim(centerShot3, shooter, pivot),
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
                                        AutoCommons.followPathFlipped(centerToWingStop, drive.translationSubsystem)
                                    )
                                )
                            )
                        ),
                        Commands.print("[Source4Note] Shot Shot Center 3")
                    )
                ;
            }
        );
    }
}
