// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.Optional;
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

  private final LoggedTunableNumber reverseIntakeSpeedThreshold = new LoggedTunableNumber("Intake/Intake Threshold", 0.1);

  private final LoggedTunableNumber intakingPopupRollerVoltage = new LoggedTunableNumber("Intake/Intaking/Pop-Up Roller Voltage", 6.0);
  private final LoggedTunableNumber intakingIntakeRollerVoltage = new LoggedTunableNumber("Intake/Intaking/Intake Roller Voltage", 6.0);
  private final LoggedTunableNumber feedingPopupRollerVoltage = new LoggedTunableNumber("Intake/Feeding/Pop-Up Roller Voltage", 0.0);
  private final LoggedTunableNumber feedingIntakeRollerVoltage = new LoggedTunableNumber("Intake/Feeding/Intake Roller Voltage", 6);
  private final LoggedTunableNumber kickingIntakeRollerVoltage = new LoggedTunableNumber("Intake/Kicking/Intake Roller Voltage", 6.0);
  private final LoggedTunableNumber kickingKickerRollerVoltage = new LoggedTunableNumber("Intake/Kicking/Kicker Roller Voltage", 6.0);

  private boolean intakeReversed;

  public static enum IntakeCommand {
    DEFAULT,
    INTAKE,
    OUTTAKE,
    FEED_TO_KICKER,
    KICK_TO_SHHOTER
  };

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    SmartDashboard.putData("Subsystems/Intake", this);
  }

  public boolean hasNote() {
    return inputs.intakeSensor || inputs.kickerSensor;
  }

  private void startIntake() {
    intakeIO.setIntakeVoltage(intakingIntakeRollerVoltage.get());
    intakeIO.setPopUpVoltage(intakingPopupRollerVoltage.get() * (intakeReversed ? -1 : 1));
  }

  private void startFeed() {
    intakeIO.setIntakeVoltage(feedingIntakeRollerVoltage.get());
    intakeIO.setPopUpVoltage(feedingPopupRollerVoltage.get() * (intakeReversed ? -1 : 1));
  }

  private void startOuttake() {
    intakeIO.setIntakeVoltage(-intakingIntakeRollerVoltage.get());
    intakeIO.setPopUpVoltage(-intakingPopupRollerVoltage.get() * (intakeReversed ? -1 : 1));
  }

  private void startKick() {
    intakeIO.setIntakeVoltage(kickingIntakeRollerVoltage.get());
    intakeIO.setKickerVoltage(kickingKickerRollerVoltage.get());
  }

  private void stopIntake() {
    intakeIO.setIntakeVoltage(0);
    intakeIO.setPopUpVoltage(0);
    intakeIO.setKickerVoltage(0);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command feedToKicker() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        startFeed();
      },
      (interrupted) -> {},
      () -> inputs.kickerSensor,
      this
    ).withName(IntakeCommand.FEED_TO_KICKER.name()).asProxy();
  }

  public Command kickToShooter() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        startKick();
      },
      (interrupted) -> {},
      () -> false,
      this
    ).withName(IntakeCommand.KICK_TO_SHHOTER.name()).asProxy();
  }

  public Command intake(Supplier<ChassisSpeeds> driveSpeedRobotRelative) {
    return new FunctionalCommand(
      () -> {
        intakeReversed = false;
      },
      () -> {
        var velocityX = driveSpeedRobotRelative.get().vxMetersPerSecond;
        if (velocityX < -reverseIntakeSpeedThreshold.get()) {
          intakeReversed = true;
        }
        if (velocityX > reverseIntakeSpeedThreshold.get()) {
          intakeReversed = false;
        }    
        startIntake();
      },
      (interrupted) -> {},
      () -> inputs.intakeSensor,
      this
    ).withName(IntakeCommand.INTAKE.name()).asProxy();
  }

  public Command outtake() {
    return new StartEndCommand(
      () -> {
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
        runDefaultCommand();
      },
      () -> {
        runDefaultCommand();
      },
      (interrupted) -> {},
      () -> false,
      this
    ).withName(IntakeCommand.DEFAULT.name());
  }

  private void runDefaultCommand() {
    stopIntake();
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
