package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
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

    public static boolean readyToShoot(Shooter shooter, Pivot pivot) {
        return shooter.readyToShoot() && pivot.atPos();
    }

    public static Command shootWhenReady(Shooter shooter, Pivot pivot, Kicker kicker) {
        return Commands.waitUntil(() -> readyToShoot(shooter, pivot)).andThen(kicker.kick());
    }

    public static Command autoAim(Drive drive, Shooter shooter, Pivot pivot) {
        return autoAim(ChassisSpeeds::new, drive, shooter, pivot);
    }

    public static Command autoAim(Supplier<ChassisSpeeds> translationalSpeeds, Drive drive, Shooter shooter, Pivot pivot) {
        Supplier<Translation2d> shootAtPos = () -> {
            var speakerTrans = AllianceFlipUtil.apply(FieldConstants.speakerAimPoint);
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

        return
            shooter.shoot(shootAtPos).asProxy()
            .deadlineWith(
                new FieldOrientedDrive(
                    drive,
                    translationalSpeeds,
                    FieldOrientedDrive.pidControlledHeading(
                        FieldOrientedDrive.pointTo(
                            () -> Optional.of(shootAtPos.get()),
                            () -> RobotConstants.shooterForward
                        )
                    )
                ),
                pivot.autoAim(shootAtPos).asProxy()
            )
            .withName("Auto Aim")
        ;
    }
}