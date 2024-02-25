package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotState;
import frc.robot.SuperCommands;
import frc.robot.auto.AutoSelector.AutoQuestion;
import frc.robot.auto.AutoSelector.AutoRoutine;
import frc.robot.commands.AutoIntake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.commands.FieldOrientedDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;

public class SpikeMarkShots extends AutoRoutine {
    private static final int MAX_QUESTION_COUNT = 3;

    private enum StartPosition {
        Speaker(FieldConstants.speakerFront),
        Amp(new Pose2d()),
        Podium(new Pose2d()),
        Source(new Pose2d())
        ;
        public final Pose2d startPose;
        StartPosition(Pose2d startPose) {
            this.startPose = startPose;
        }
    }

    private static final AutoQuestion<StartPosition> startPosition = new AutoQuestion<>("Start Position", StartPosition::values); 

    private static final RobotState robotState = RobotState.getInstance();

    public SpikeMarkShots(Drive drive, Shooter shooter, Pivot pivot, Intake intake, Kicker kicker, AutoIntake autoIntake) {
        super("Spike Mark Shots",
            MAX_QUESTION_COUNT,
            () -> {
                return List.of(startPosition);
            },
            () -> {
                return Commands.runOnce(() -> robotState.setPose(drive.getGyroRotation(), drive.getModulePositions(), AllianceFlipUtil.apply(startPosition.getResponse().startPose)))
                    .andThen(
                        SuperCommands.autoAim(drive, shooter, pivot),
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
                        drive.driveTo(FieldConstants.speakerFront),
                        Commands.waitUntil(kicker::hasNote),
                        SuperCommands.autoAim(drive, shooter, pivot)
                    )
                    .withName("AUTO Spike Mark Shots")
                ;
            }
        );

    }
    
}
