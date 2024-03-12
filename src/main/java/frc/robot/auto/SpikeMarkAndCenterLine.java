package frc.robot.auto;

import java.util.List;

import com.fasterxml.jackson.core.sym.Name;
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

public class SpikeMarkAndCenterLine extends AutoRoutine {
    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", StartPosition::values); 

    public SpikeMarkAndCenterLine(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public SpikeMarkAndCenterLine(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super("SpikeMarkAndCenterLine",
            List.of(startPosition),
            () -> {
                PathPlannerPath startToCenterLine = AutoPaths.loadPath(String.format(AutoPaths.toCenterLine, startPosition.getResponse().name()));

                return AutoCommons.setOdometryFlipped(startPosition.getResponse().startPose, drive)
                    .andThen(
                        // AutoCommons.autoAimAndFollowPath(startToCenterLine, drive, shooter, pivot, kicker)
                        // AutoCommons.autoAimAndShootWhenReady(drive, shooter, pivot, kicker),
                        // AutoCommons.followPathFlipped(startToCenterLine, drive),
                        // AutoCommons.autoIntake(1.5, drive, intake, noteVision),
                        // drive.driveToFlipped(FieldConstants.subwooferFront),
                        // AutoCommons.autoAimAndShootWhenReady(drive, shooter, pivot, kicker)
                    )
                ;
            }
        );
    }
}
