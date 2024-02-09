// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final LoggedTunableNumber maxRPS = new LoggedTunableNumber("Shooter/Max Rotations Per Second", 0);
  private final LoggedTunableNumber spinRPS = new LoggedTunableNumber("Shooter/Spin in Rotations Per Second", 0);

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    SmartDashboard.putData("Subsystems/Shooter", this);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command shoot() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        if (spinRPS.get() >= 0) {
          shooterIO.setLeftVelocity(maxRPS.get());
          shooterIO.setRightVelocity(maxRPS.get() - spinRPS.get());
        } else if (spinRPS.get() < 0) {
          shooterIO.setLeftVelocity(maxRPS.get() + spinRPS.get());
          shooterIO.setRightVelocity(maxRPS.get());
        }
      },
      (interrupted) -> {},
      () -> !inputs.notePresent, // detect falling edge and confirm with rps dip from the motors
      this
    );
  }
}
