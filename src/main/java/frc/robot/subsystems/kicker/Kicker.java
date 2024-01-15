// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Kicker extends SubsystemBase {
  private final KickerIO kickerIO;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  private final LoggedTunableNumber kickerVolts = new LoggedTunableNumber("Kicker/Kicker Voltage", 2);

  public Kicker(KickerIO kickerIO) {
    this.kickerIO = kickerIO;    
  }

  @Override
  public void periodic() {
    kickerIO.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }

  public Command feedToShooter() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        kickerIO.setKickerVoltage(kickerVolts.get());
      },
      (interrupted) -> {
        kickerIO.setKickerVoltage(0);
      },
      () -> !inputs.notePresent,
      this
    ).withName("Kicker/FeedToShooter");
  }
}
