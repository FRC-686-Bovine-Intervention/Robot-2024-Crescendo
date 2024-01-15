// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

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

  private final LoggedTunableNumber intakeVoltage = new LoggedTunableNumber("Intake/Intake Voltage", 5);
  private final LoggedTunableNumber beltVoltage = new LoggedTunableNumber("Intake/Belt Voltage", 5);
  private final LoggedTunableNumber reverseSpeedThresold = new LoggedTunableNumber("Intake/Reverse Speed Threshold", 0.1);

  private boolean intakeReversed;

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
    SmartDashboard.putData("Subsystems/Intake", this);
  }

  public boolean hasNote() {
    return inputs.noteAtBottom || inputs.noteAtTop;
  }

  public boolean noteReady() {
    return inputs.noteAtTop;
  }

  private void startIntake() {
    intakeIO.setBeltVoltage(beltVoltage.get());
    intakeIO.setIntakeVoltage(intakeVoltage.get() * (intakeReversed ? -1 : 1));
  }

  private void startOuttake() {
    intakeIO.setBeltVoltage(-beltVoltage.get());
    intakeIO.setIntakeVoltage(-intakeVoltage.get() * (intakeReversed ? -1 : 1));
  }

  private void stopIntake() {
    intakeIO.setBeltVoltage(0);
    intakeIO.setIntakeVoltage(0);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command secureNote() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        startIntake();
      },
      (interrupted) -> {},
      () -> inputs.noteAtTop,
      this
    ).withName("Secure Note");
  }

  public Command feedToKicker() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        startIntake();
      },
      (interrupted) -> {},
      () -> !inputs.noteAtTop,
      this
    ).withName("Feed");
  }

  public Command intake(Supplier<ChassisSpeeds> driveSpeedRobotRelative) {
    return new FunctionalCommand(
      () -> {
        intakeReversed = false;
      },
      () -> {
        var velocityX = driveSpeedRobotRelative.get().vxMetersPerSecond;
        if (velocityX < -reverseSpeedThresold.get()) {
          intakeReversed = true;
        }
        if (velocityX > reverseSpeedThresold.get()) {
          intakeReversed = false;
        }    
        startIntake();
      },
      (interrupted) -> {},
      () -> inputs.noteAtTop,
      this
    ).withName("Intake");
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
    ).withName("Outtake");
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
    ).withName("Default");
  }

  private void runDefaultCommand() {
    if (inputs.noteAtBottom && !inputs.noteAtTop) {
      secureNote().schedule();
    }

    stopIntake();
  }
}
