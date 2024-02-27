package frc.robot.auto;

import java.util.List;
import java.util.function.Function;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.SuperCommands;
import frc.robot.Constants.FieldConstants;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.commands.AutoIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.commands.FieldOrientedDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;

public class CenterLineRun extends AutoRoutine {
    private static final int MAX_QUESTION_COUNT = 3;

    private enum StartPosition {
        Speaker,
        Amp,
        Podium,
        Source
    }

    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", StartPosition::values); 

    private static final RobotState robotState = RobotState.getInstance();

    private static final String startTemplate = "%s Start";
    private static final String backTemplate = "%s Back";

    public CenterLineRun(Drive drive, Shooter shooter, Pivot pivot, Intake intake, AutoIntake autoIntake) {
        super("Center Line Run",
            MAX_QUESTION_COUNT,
            () -> {
                return List.of(startPosition);
            },
            () -> {
                Function<PathPlannerPath, Command> followPathConstructor = (path) -> new FollowPathHolonomic(path, robotState::getPose, drive::getChassisSpeeds, drive::driveVelocity, Drive.autoConfigSup.get(), AllianceFlipUtil::shouldFlip, drive);
                
                PathPlannerPath startToNote = PathPlannerPath.fromPathFile(String.format(startTemplate, startPosition.getResponse().toString()));
                PathPlannerPath noteToStart = PathPlannerPath.fromPathFile(String.format(backTemplate, startPosition.getResponse().toString()));

                return Commands.runOnce(() -> robotState.setPose(drive.getGyroRotation(), drive.getModulePositions(), AllianceFlipUtil.apply(new Pose2d(startToNote.getPoint(0).position, Rotation2d.fromDegrees(180)))))
                    .andThen(
                        SuperCommands.autoAim(drive, shooter, pivot),
                        followPathConstructor.apply(startToNote),
                        intake.intake(drive::getChassisSpeeds).asProxy().deadlineWith(new FieldOrientedDrive(
                            drive,
                            autoIntake.getTransSpeed(() -> 1.5).orElseGet(() -> new ChassisSpeeds(0, -1, 0)),
                            FieldOrientedDrive.pidControlledHeading(
                                FieldOrientedDrive.pointTo(
                                    autoIntake.targetLocation(),
                                    () -> Rotation2d.fromDegrees(180)
                                )
                            )
                        )),
                        // followPathConstructor.apply(noteToStart),
                        drive.driveToFlipped(FieldConstants.speakerFront),
                        SuperCommands.autoAim(drive, shooter, pivot)
                    )
                    .withName("AUTO Center Line Run")
                ;
            }
        );

    }
    
}
