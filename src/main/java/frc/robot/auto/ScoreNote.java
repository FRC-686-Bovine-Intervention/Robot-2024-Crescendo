package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicStampedReference;
import java.util.function.Function;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.SuperCommands;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.commands.AutoIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;

public class ScoreNote extends AutoRoutine {
    private static final int MAX_QUESTION_COUNT = 3;

    private enum StartPosition {
        Amp,
        Speaker,
        Podium,
        Source
    }

    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", StartPosition::values); 

    private static final RobotState robotState = RobotState.getInstance();

    private static final String startTemplate = "%s Start";

    public ScoreNote(Drive drive, Shooter shooter, Pivot pivot) {
        super("ScoreNote",
            MAX_QUESTION_COUNT,
            () -> {
                return List.of(startPosition);
            },
            () -> {
                Function<PathPlannerPath, Command> followPathConstructor = (path) -> new FollowPathHolonomic(path, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, Drive.autoConfigSup.get(), AllianceFlipUtil::shouldFlip, drive);
                
                PathPlannerPath startToNote = PathPlannerPath.fromPathFile(String.format(startTemplate, startPosition.getResponse().toString()));

                return Commands.runOnce(() -> robotState.setPose(drive.getGyroRotation(), drive.getModulePositions(), AllianceFlipUtil.apply(new Pose2d(startToNote.getPoint(0).position, new Rotation2d()))))
                    .andThen(SuperCommands.autoAim(drive, shooter, pivot))
                    .andThen(followPathConstructor.apply(startToNote))
                    .andThen(Commands.none()); // autointake
            }
        );

    }
    
}
