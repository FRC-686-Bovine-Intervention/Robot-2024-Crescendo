// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Kicker extends SubsystemBase {
  private final KickerIO kickerIO;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  private final LoggedTunableNumber kickerVolts = new LoggedTunableNumber("Kicker/Kicker Voltage", 5);
  private final LoggedTunableNumber feedVolts = new LoggedTunableNumber("Kicker/Feed Voltage", 1.5);
  private final LoggedTunableNumber antiDeadzoneVolts = new LoggedTunableNumber("Kicker/Anti Deadzone Voltage", 1.5);

  public Kicker(KickerIO kickerIO) {
    this.kickerIO = kickerIO;
    SmartDashboard.putData("Subsystems/Kicker", this);   
  }

  @Override
  public void periodic() {
    kickerIO.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public Command feedIn() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        kickerIO.setKickerVoltage(feedVolts.get());
      },
      (interrupted) -> {
        kickerIO.setKickerVoltage(0);
      },
      () -> inputs.notePresent,
      this
    ).withName("Feed In");
  }

  public Command kick() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        kickerIO.setKickerVoltage(kickerVolts.get());
      },
      (interrupted) -> {
        kickerIO.setKickerVoltage(0);
      },
      () -> false,
      this
    ).withName("Kick");
  }

  public Command outtake() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        kickerIO.setKickerVoltage(-kickerVolts.get());
      },
      (interrupted) -> {
        kickerIO.setKickerVoltage(0);
      },
      () -> false,
      this
    ).withName("Outtake");
  }

  public Command antiDeadzone() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        kickerIO.setKickerVoltage(antiDeadzoneVolts.get());
      },
      (interrupted) -> {
        kickerIO.setKickerVoltage(0);
      },
      () -> false,
      this
    ).withName("Anti Deadzone|");
  }

  public Command doNothing() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        kickerIO.setKickerVoltage(0);
      },
      (interrupted) -> {
        kickerIO.setKickerVoltage(0);
      },
      () -> false,
      this
    ).withName("Do Nothing|");
  }

  public boolean hasNote() {
    return inputs.notePresent;
  }
}
