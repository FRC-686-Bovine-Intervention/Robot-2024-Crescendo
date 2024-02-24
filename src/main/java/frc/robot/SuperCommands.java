package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.commands.FieldOrientedDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.MathExtraUtil;

public class SuperCommands {
    public static Command feedToKicker(Intake intake, Kicker kicker) {
        return 
            intake.feedToKicker(kicker::hasNote).asProxy()
            .alongWith(kicker.feedIn())
            .withName("Feed Into Kicker")
        ;
    }

    public static Command autoAim(Drive drive, Shooter shooter, Pivot pivot) {
        Supplier<Translation2d> shootAtPos = () -> {
            var speakerTrans = AllianceFlipUtil.apply(FieldConstants.speakerCenter);
            var chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(drive.getChassisSpeeds(), drive.getPose().getRotation());
            var robotToSpeaker = speakerTrans.minus(drive.getPose().getTranslation());
            var robotToSpeakerNorm = robotToSpeaker.div(robotToSpeaker.getNorm());
            var velocityTowardsSpeaker = MathExtraUtil.dotProduct(robotToSpeakerNorm, new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
            var timeToSpeaker = drive.getPose().getTranslation().getDistance(speakerTrans) / (ShooterConstants.exitVelocity + velocityTowardsSpeaker);
            var chassisOffset = chassisSpeeds.times(timeToSpeaker);
            var translationalOffset = new Translation2d(chassisOffset.vxMetersPerSecond, chassisOffset.vyMetersPerSecond);
            var pointTo = speakerTrans.minus(translationalOffset);
            return pointTo;
        };

        return shooter.shoot(shootAtPos).asProxy().deadlineWith(
                new FieldOrientedDrive(
                    drive,
                    () -> new ChassisSpeeds(),
                    FieldOrientedDrive.pidControlledHeading(
                        FieldOrientedDrive.pointTo(
                            () -> Optional.of(shootAtPos.get()),
                            () -> Rotation2d.fromDegrees(0)
                        )
                    )
                ),
                pivot.autoAim(shootAtPos).asProxy()
            )
            .withName("AutoAim");
    }
}