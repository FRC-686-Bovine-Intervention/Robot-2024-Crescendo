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
            .alongWith(kicker.feedIn().asProxy())
            .withName("Feed Into Kicker")
        ;
    }

    public static boolean readyToShoot(Shooter shooter, Pivot pivot) {
        return shooter.readyToShoot() && pivot.readyToShoot();
    }

    public static Command shootWhenReady(Shooter shooter, Pivot pivot, Kicker kicker) {
        return Commands.waitUntil(() -> kicker.hasNote() && readyToShoot(shooter, pivot)).andThen(kicker.kick().asProxy());
    }

    // public static Supplier<Translation2d> autoAimFORR(Drive drive) {
    //     return autoAimFORR(() -> drive.getPose().getTranslation(), () -> ChassisSpeeds.fromRobotRelativeSpeeds(drive.getRobotRelativeSpeeds(), drive.getRotation()));
    // }

    // public static Command autoAim(Supplier<Translation2d> FORR, Drive.Rotational rotation, Shooter shooter, Kicker kicker, Pivot pivot) {
    //     return
    //         shooter.shoot(FORR, kicker::sensorFallingEdge).asProxy()
    //         .deadlineWith(
    //             rotation.pidControlledHeading(
    //                 () -> {
    //                     var t = FORR.get();
    //                     return Optional.of(new Rotation2d(t.getX(), t.getY()));
    //                 }
    //             ).withName("Auto Aim").asProxy(),
    //             pivot.autoAim(FORR).asProxy()
    //         )
    //     ;
    // }



    // public static Command autoAim(Drive.Rotational rotation, Shooter shooter, Kicker kicker, Pivot pivot) {
    //     return autoAim(autoAimFORR(rotation.drive), rotation, shooter, kicker, pivot);
    // }
}