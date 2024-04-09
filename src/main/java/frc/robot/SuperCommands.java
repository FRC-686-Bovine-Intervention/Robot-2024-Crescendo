package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

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
}