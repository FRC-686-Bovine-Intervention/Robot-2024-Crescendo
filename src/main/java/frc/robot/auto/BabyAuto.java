package frc.robot.auto;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

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

public class BabyAuto extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", StartPosition::values);
    private static final AutoQuestion<Spike> spike = new AutoQuestion<>("Spike", Spike::values);

    private static enum Spike {
        Yes,
        No
    }

    private static PathPlannerPath getSpikePath() {
        switch(startPosition.getResponse()) {
            default:
            case Amp:
            case SubwooferAmp:
                return (AutoPaths.loadPath(String.format(AutoPaths.startToSpike, "Amp")));
            case SubwooferFront:
                return (AutoPaths.loadPath(String.format(AutoPaths.startToSpike, "SubwooferFront")));
            case Source:
            case SubwooferSource:
                return (AutoPaths.loadPath(String.format(AutoPaths.startToSpike, "Podium")));
        }
    }

    public BabyAuto(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public BabyAuto(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super("Baby Auto",
            List.of(startPosition, spike),
            () -> {
                PathPlannerPath startToSpike = getSpikePath();

                var preloadShot = AllianceFlipUtil.apply(startPosition.getResponse().startPose.getTranslation());
                var spikeShot = AllianceFlipUtil.apply(startToSpike.getPoint(startToSpike.numPoints() - 1).position);

                var command = AutoCommons.setOdometryFlipped(startPosition.getResponse().startPose, drive)
                .andThen(
                    AutoCommons.shootWhenReady(preloadShot, drive, shooter, pivot, kicker)
                    .deadlineWith(
                        AutoCommons.autoAim(preloadShot, shooter, kicker, pivot, drive.rotationalSubsystem)
                    )
                );
                switch (spike.getResponse()) {
                    default:
                    case Yes:
                        command = command.andThen(
                            AutoCommons.shootWhenReady(spikeShot, drive, shooter, pivot, kicker)
                            .deadlineWith(
                                intake.intake(drive::getChassisSpeeds),
                                AutoCommons.autoAim(spikeShot, shooter, kicker, pivot, drive.rotationalSubsystem),
                                AutoCommons.followPathFlipped(startToSpike, drive.translationSubsystem)
                            )
                        );
                    break;
                    case No:
                    break;
                }
                return command;
            }
        );
    }
}
