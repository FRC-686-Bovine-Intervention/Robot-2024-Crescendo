// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    System.out.println("[Init Robot] Recording AdvantageKit Metadata");
    Logger.recordMetadata("Robot", RobotType.getRobot().name());
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
      break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
      break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
      break;
    }

    // Set up data receivers & replay source
    System.out.println("[Init Robot] Configuring AdvantageKit for " + RobotType.getMode().name() + " " + RobotType.getRobot().name());
    switch (RobotType.getMode()) {
      // Running on a real robot, log to a USB stick
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
        Logger.addDataReceiver(new NT4Publisher());
      break;

      // Running a physics simulator, log to local folder
      case SIM:
        Logger.addDataReceiver(new WPILOGWriter(""));
        Logger.addDataReceiver(new NT4Publisher());
      break;

      // Replaying a log, set up replay source
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      break;
    }

    System.out.println("[Init Robot] Starting AdvantageKit");
    Logger.start();

    System.out.println("[Init Robot] Starting Command Logger");
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
      (Command command, Boolean active) -> {
        String name = command.getName();
        int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
        commandCounts.put(name, count);
        // Logger.recordOutput(
        //         "Commands/Unique/" + name + "_" + Integer.toHexString(command.hashCode()), active.booleanValue());
        Logger.recordOutput("Commands/All/" + name, count > 0);
    };

    CommandScheduler.getInstance()
      .onCommandInitialize(
        (Command command) -> {
          logCommandFunction.accept(command, true);
        }
      );
    CommandScheduler.getInstance()
      .onCommandFinish(
        (Command command) -> {
          logCommandFunction.accept(command, false);
        }
      );
    CommandScheduler.getInstance()
      .onCommandInterrupt(
        (Command command) -> {
          logCommandFunction.accept(command, false);
        }
      );

    System.out.println("[Init Robot] Instantiating RobotContainer");
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.robotPeriodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
