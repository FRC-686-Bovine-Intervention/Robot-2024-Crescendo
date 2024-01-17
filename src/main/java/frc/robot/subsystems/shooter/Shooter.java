// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
  
  private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/PID/kP", 0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Shooter/PID/kI", 0);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/PID/kD", 0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/PID/kV", 0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/PID/kA", 0);
    
  private final ProfiledPIDController motorPID = new ProfiledPIDController(
      kP.get(),
      kI.get(),
      kD.get(),
      new Constraints(
          kV.get(),
          kA.get()
      )
  );

  private void updateTunables() {
      if(kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
          motorPID.setPID(kP.get(), kI.get(), kD.get());
      }
      if(kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
          motorPID.setConstraints(new Constraints(kV.get(), kA.get()));
      }
  }

  public Shooter(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    SmartDashboard.putData("Subsystems/Shooter", this);
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    updateTunables();
  }

  public Command shoot() {
    return new FunctionalCommand(
      () -> {},
      () -> {
        var maxVoltage = motorPID.calculate(inputs.leftRotationsPerSecond, maxRPS.get());
        var spinVoltage = motorPID.calculate(inputs.rightRotationsPerSecond, maxRPS.get() - Math.abs(spinRPS.get()));
        if (spinRPS.get() >= 0) {
          shooterIO.setLeftVoltage(maxVoltage);
          shooterIO.setRightVoltage(spinVoltage);
        } else if (spinRPS.get() < 0) {
          shooterIO.setLeftVoltage(spinVoltage);
          shooterIO.setRightVoltage(maxVoltage);
        }
      },
      (interrupted) -> {},
      () -> !inputs.notePresent,
      this
    );
  }
}
