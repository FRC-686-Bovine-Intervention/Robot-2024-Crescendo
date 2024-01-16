// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    SmartDashboard.putData("Subsystems/Shooter", this);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
