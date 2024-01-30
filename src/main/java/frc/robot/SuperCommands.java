package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.kicker.Kicker;

public class SuperCommands {
    public static Command feedToKicker(Intake intake, Kicker kicker) {
        return 
            intake.feedToKicker()
            .alongWith(kicker.feedIn())
            .withName("Feed Into Kicker")
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public static Command autoAim(Drive drive) {

        return Commands.none();
    }
}
