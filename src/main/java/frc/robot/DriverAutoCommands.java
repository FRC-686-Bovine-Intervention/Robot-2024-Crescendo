package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriverAutoCommands {
    private static final PathPlannerPath testPath = PathPlannerPath.fromPathFile("New Path");

    public static Command followTestPath(Drive drive) {
        return
            drive.followPath(testPath);
    }
}
