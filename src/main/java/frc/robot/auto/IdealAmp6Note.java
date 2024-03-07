package frc.robot.auto;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.RobotContainer;
import frc.robot.auto.AutoCommons.AutoPaths;
import frc.robot.auto.AutoCommons.StartPosition;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.note.NoteVision;

public class IdealAmp6Note extends AutoRoutine {
    public IdealAmp6Note(RobotContainer robot) {
        this(robot.drive, robot.shooter, robot.pivot, robot.kicker, robot.intake, robot.noteVision);
    }
    public IdealAmp6Note(Drive drive, Shooter shooter, Pivot pivot, Kicker kicker, Intake intake, NoteVision noteVision) {
        super("IdealAmp6Note",
            List.of(),
            () -> {
                PathPlannerPath startToSpike = AutoPaths.loadPath(String.format(AutoPaths.startToSpike, "Amp"));


                // Shoot preload and start driving to spike note
                // Intake spike note and (preemptive?) move to center
                // AutoIntake with sweep
                // Drive back to wing and shoot
                // AutoIntake with sweep
                // Drive back to wing and continue to center spike
                return AutoCommons.setOdometryFlipped(StartPosition.Amp.startPose, drive)
                    .andThen(
                        // AutoCommons.autoAimAndFollowPath(startToSpike, drive, shooter, pivot, kicker),
                        // AutoCommons.autoAimAndFollowPath(startToSpike, drive, shooter, pivot, kicker)
                        // .alongWith(
                        //     intake.intake(drive::getChassisSpeeds)
                        // )
                    )
                ;
            }
        );
    }
}
