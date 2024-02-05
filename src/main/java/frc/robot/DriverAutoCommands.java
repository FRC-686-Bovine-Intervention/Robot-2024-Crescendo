package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;

public class DriverAutoCommands {
    private static final PathPlannerPath testPath = PathPlannerPath.fromPathFile("New Path");

    public static Command followTestPath(Drive drive) {
        return Commands.none();
    }
}
