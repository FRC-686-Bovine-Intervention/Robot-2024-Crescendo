// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableNumber;

public class AutoDrive extends Command {
  private final Supplier<Pose2d> fieldPos;
  
  private Pose2d lastFieldPos;
  private Command pathFindCommand;

  private static final LoggedTunableNumber tP = new LoggedTunableNumber("AutoDrive/tP", 1);
  private static final LoggedTunableNumber tI = new LoggedTunableNumber("AutoDrive/tI", 0);
  private static final LoggedTunableNumber tD = new LoggedTunableNumber("AutoDrive/tD", 0);
  private static final LoggedTunableNumber rP = new LoggedTunableNumber("AutoDrive/rP", 1.5);
  private static final LoggedTunableNumber rI = new LoggedTunableNumber("AutoDrive/rI", 0);
  private static final LoggedTunableNumber rD = new LoggedTunableNumber("AutoDrive/rD", 0);
  private static final Supplier<HolonomicPathFollowerConfig> configSup = () -> {
      return new HolonomicPathFollowerConfig(
          new PIDConstants(
              tP.get(),
              tI.get(),
              tD.get()
          ),
          new PIDConstants(
              rP.get(),
              rI.get(),
              rD.get()
          ),
          DriveConstants.maxDriveSpeedMetersPerSec,
          0.46,
          new ReplanningConfig()
      );
  };

    public static final LoggedTunableNumber kAutoDriveMaxVelocity = new LoggedTunableNumber("Drive/kAutoDriveMaxVelocity", 3.0);
    public static final LoggedTunableNumber kMaxAcceleration = new LoggedTunableNumber("Drive/kMaxAcceleration", 4.0);
    public static final LoggedTunableNumber kMaxAngularVelocity = new LoggedTunableNumber("Drive/kMaxAngularVelocity", Units.degreesToRadians(540));
    public static final LoggedTunableNumber kMaxAngularAcceleration = new LoggedTunableNumber("Drive/kMaxAngularAcceleration", Units.degreesToRadians(720));
    private static final PathConstraints pathConstraints = new PathConstraints(
        kAutoDriveMaxVelocity.get(),
        kMaxAcceleration.get(),
        kMaxAngularVelocity.get(),
        kMaxAngularAcceleration.get()
    );

  private Command driveTo(Pose2d pos) {
    return AutoBuilder.pathfindToPose(pos, pathConstraints, 0, 0);
  }

  public AutoDrive(Drive drive, Supplier<Pose2d> fieldPos, BooleanSupplier shouldInterruptCommand) {
    this.fieldPos = fieldPos;

    if (!AutoBuilder.isConfigured()) {
      AutoBuilder.configureHolonomic(
          drive::getPose,
          drive::setPose,
          drive::getChassisSpeeds,
          drive::driveVelocity,
          configSup.get(),
          AllianceFlipUtil::shouldFlip,
          drive
      );
    }

    this.pathFindCommand = driveTo(this.fieldPos.get());
    this.lastFieldPos = fieldPos.get();

    new Trigger(shouldInterruptCommand).onTrue(Commands.runOnce(() -> {
      cancel();
      pathFindCommand.cancel();
    }));
  }
  
  @Override
  public void execute() {
    if (
      !lastFieldPos.equals(fieldPos.get()) ||
      pathFindCommand == null
    ) {
      pathFindCommand = driveTo(this.fieldPos.get());
      lastFieldPos = fieldPos.get();
    }

    if (pathFindCommand != null) {
      if (
        !pathFindCommand.isFinished() &&
        !pathFindCommand.isScheduled()
      ) {
        pathFindCommand.schedule();
      } else {
        if (pathFindCommand.isFinished()) {
          pathFindCommand = null;
        }
      }
    }
  }
}
