// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final LoggedTunableNumber intakingRollerVoltage = new LoggedTunableNumber("Intake/Intaking/Roller Voltage", 6);
  private final LoggedTunableNumber intakingBeltVoltage = new LoggedTunableNumber("Intake/Intaking/Belt Voltage", 6);
  private final LoggedTunableNumber feedingRollerVoltage = new LoggedTunableNumber("Intake/Feeding/Roller Voltage", 3);
  private final LoggedTunableNumber feedingBeltVoltage = new LoggedTunableNumber("Intake/Feeding/Belt Voltage", 3);
  private final LoggedTunableNumber antiDeadzoneBeltVoltage = new LoggedTunableNumber("Intake/Anti Deadzone/Belt Voltage", 1);
  private final LoggedTunableNumber reverseSpeedThresold = new LoggedTunableNumber("Intake/Reverse Speed Threshold", 0.1);

  private boolean intakeReversed;

  public static enum IntakeCommand {
    NOTHING,
    ANTI_DEADZONE,
    INTAKE,
    OUTTAKE,
    FEED_TO_KICKER,
  };

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    SmartDashboard.putData("Subsystems/Intake", this);
  }

  public boolean hasNote() {
    return inputs.sensor;
  }

  private void startIntake() {
    intakeIO.setBeltVoltage(intakingBeltVoltage.get());
    intakeIO.setRollerVoltage(intakingRollerVoltage.get() * (intakeReversed ? -1 : 1));
  }

  private void startFeed() {
    intakeIO.setBeltVoltage(feedingBeltVoltage.get());
    intakeIO.setRollerVoltage(feedingRollerVoltage.get() * (intakeReversed ? -1 : 1));
  }

  private void startOuttake() {
    intakeIO.setBeltVoltage(-intakingBeltVoltage.get());
    intakeIO.setRollerVoltage(-intakingRollerVoltage.get() * (intakeReversed ? -1 : 1));
  }

  private void startAntiDeadzone() {
    intakeIO.setBeltVoltage(antiDeadzoneBeltVoltage.get());
    intakeIO.setRollerVoltage(0);
  }

  private void stopIntake() {
    intakeIO.setBeltVoltage(0);
    intakeIO.setRollerVoltage(0);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command feedToKicker(BooleanSupplier kickerSensor) {
    return new FunctionalCommand(
      () -> {},
      () -> {
        startFeed();
      },
      (interrupted) -> {},
      () -> kickerSensor.getAsBoolean(),
      this
    ).withName(IntakeCommand.FEED_TO_KICKER.name()).asProxy();
  }

  public Command intake(Supplier<ChassisSpeeds> driveSpeedRobotRelative) {
    return new FunctionalCommand(
      () -> {
        intakeReversed = false;
      },
      () -> {
        var velocityX = driveSpeedRobotRelative.get().vxMetersPerSecond;
        if (velocityX < -reverseSpeedThresold.get()) {
          intakeReversed = false;
        }
        if (velocityX > reverseSpeedThresold.get()) {
          intakeReversed = true;
        }    
        startIntake();
      },
      (interrupted) -> {},
      () -> inputs.sensor,
      this
    ).withName(IntakeCommand.INTAKE.name()).asProxy();
  }

  public Command outtake(Supplier<ChassisSpeeds> driveSpeedRobotRelative) {
    return new StartEndCommand(
      () -> {
        var velocityX = driveSpeedRobotRelative.get().vxMetersPerSecond;
        if (velocityX < -reverseSpeedThresold.get()) {
          intakeReversed = true;
        }
        if (velocityX > reverseSpeedThresold.get()) {
          intakeReversed = false;
        }    
        startOuttake();
      },
      () -> {
        stopIntake();
      },
      this
    ).withName(IntakeCommand.OUTTAKE.name()).asProxy();
  }

  public Command doNothing() {
    return new FunctionalCommand(
      () -> {
        stopIntake();
      },
      () -> {
        stopIntake();
      },
      (interrupted) -> {},
      () -> false,
      this
    ).withName(IntakeCommand.NOTHING.name());
  }

  public Command antiDeadzone() {
    var subsystem = this;
    return new Command() {
      {
        addRequirements(subsystem);
        setName(IntakeCommand.ANTI_DEADZONE.name());
      }
      @Override
      public void initialize() {
        startAntiDeadzone();
      }
      @Override
      public void execute() {
        startAntiDeadzone();
      }
      @Override
      public void end(boolean interrupted) {
        stopIntake();
      }
    };
  }

  public boolean getIntakeReversed() {
    return intakeReversed;
  }

  public Optional<IntakeCommand> getIntakeCommand() {
    return Optional.ofNullable(getCurrentCommand()).map(Command::getName).flatMap((n) -> {
      try {
        return Optional.of(IntakeCommand.valueOf(n));
      } catch (Exception e) {
        return Optional.empty();
      }
    });
  }
}
