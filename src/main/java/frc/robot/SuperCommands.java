package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.AllianceFlipUtil;

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
        return Commands.waitUntil(() -> kicker.hasNote() && readyToShoot(shooter, pivot)).andThen(kicker.kick().asProxy());
    }

    // public static Supplier<Translation2d> autoAimShootAtPos(Drive drive) {
    //     return () -> {
    //         var speakerTrans = AllianceFlipUtil.apply(FieldConstants.speakerAimPoint);
    //         var chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getPose().getRotation());
    //         var robotToSpeaker = speakerTrans.minus(drive.getPose().getTranslation());
    //         var robotToSpeakerNorm = robotToSpeaker.div(robotToSpeaker.getNorm());
    //         var velocityTowardsSpeaker = MathExtraUtil.dotProduct(robotToSpeakerNorm, new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
    //         var timeToSpeaker = drive.getPose().getTranslation().getDistance(speakerTrans) / (ShooterConstants.exitVelocity + velocityTowardsSpeaker);
    //         var chassisOffset = chassisSpeeds.times(timeToSpeaker);
    //         var translationalOffset = new Translation2d(chassisOffset.vxMetersPerSecond, chassisOffset.vyMetersPerSecond);
    //         var pointTo = speakerTrans.minus(translationalOffset);
    //         Logger.recordOutput("Shooter/Shoot at", pointTo);
    //         return pointTo;
    //     };
    // }

    public static Supplier<Translation2d> autoAimFORR(Drive drive) {
        return autoAimFORR(() -> drive.getPose().getTranslation(), () -> ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation()));
    }

    public static Supplier<Translation2d> autoAimFORR(Supplier<Translation2d> robotTranslation, Supplier<ChassisSpeeds> robotVelocityFieldRel) {
        return () -> {
            var robotTrans = robotTranslation.get();
            var speakerTrans = AllianceFlipUtil.apply(FieldConstants.speakerAimPoint);
            var chassisSpeeds = robotVelocityFieldRel.get();
            var robotToSpeaker = speakerTrans.minus(robotTrans);
            var robotToSpeakerNorm = robotToSpeaker.div(robotToSpeaker.getNorm());
            var velocityTowardsSpeaker = robotToSpeakerNorm.toVector().dot(VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));
            var timeToSpeaker = robotTrans.getDistance(speakerTrans) / (ShooterConstants.exitVelocity + velocityTowardsSpeaker);
            var chassisOffset = chassisSpeeds.times(timeToSpeaker);
            var translationalOffset = new Translation2d(chassisOffset.vxMetersPerSecond, chassisOffset.vyMetersPerSecond);
            var pointTo = speakerTrans.minus(translationalOffset);
            Logger.recordOutput("Shooter/Shoot at", pointTo);
            return pointTo.minus(robotTrans);
        };
    }

    public static Command autoAim(Supplier<Translation2d> FORR, Drive.Rotational rotation, Shooter shooter, Pivot pivot) {
        return
            shooter.shoot(FORR).asProxy()
            .deadlineWith(
                rotation.pidControlledHeading(
                    () -> {
                        var t = FORR.get();
                        return Optional.of(new Rotation2d(t.getX(), t.getY()));
                    }
                ),
                pivot.autoAim(FORR).asProxy()
            )
            .withName("Auto Aim")
        ;
    }

    public static Command autoAim(Drive.Rotational rotation, Shooter shooter, Pivot pivot) {
        return autoAim(autoAimFORR(rotation.drive), rotation, shooter, pivot);
    }
}