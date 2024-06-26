// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers.intake;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final LoggedTunableNumber toggleReverseThreshold = new LoggedTunableNumber("Intake/Toggle Reverse Threshold", 0.1);
  private final DoubleSupplier forwardSpeedSupplier;
  
  private boolean intakeReversed;

  public Intake(IntakeIO intakeIO, Supplier<ChassisSpeeds> robotRelativeSpeeds) {
    System.out.println("[Init Intake] Instantiating Intake");
    this.intakeIO = intakeIO;
    System.out.println("[Init Intake] Intake IO: " + this.intakeIO.getClass().getSimpleName());
    SmartDashboard.putData("Subsystems/Intake", this);
    forwardSpeedSupplier = () -> robotRelativeSpeeds.get().vxMetersPerSecond * (intakeReversed ? -1 : 1);
  }

  public Command antiDeadzone() {
    var subsystem = this;
    return new Command() {
      private final LoggedTunableNumber rollerVoltage = new LoggedTunableNumber("Intake/Anti Deadzone/Roller Voltage", 0);
      private final LoggedTunableNumber beltVoltage = new LoggedTunableNumber("Intake/Anti Deadzone/Belt Voltage", 0);
      {
        setName("AntiDeadzone");
        addRequirements(subsystem);
      }

      public void execute() {
        subsystem.intakeIO.setRollerVoltage(this.rollerVoltage.get() * (getIntakeReversed() ? -1 : 1));
        subsystem.intakeIO.setBeltVoltage(this.beltVoltage.get());
      }
    };
  }

  public Command intake() {
    var subsystem = this;
    return new Command() {
      private final LoggedTunableNumber rollerVoltage = new LoggedTunableNumber("Intake/Intaking/Roller Voltage", 6);
      private final LoggedTunableNumber beltVoltage = new LoggedTunableNumber("Intake/Intaking/Belt Voltage", 6);
      {
        setName("Intake");
        addRequirements(subsystem);
      }

      @Override
      public void execute() {
        if(subsystem.forwardSpeedSupplier.getAsDouble() >= Intake.toggleReverseThreshold.get()) {
          subsystem.intakeReversed = !subsystem.intakeReversed;
        }
        subsystem.intakeIO.setRollerVoltage(this.rollerVoltage.get() * (getIntakeReversed() ? -1 : 1));
        subsystem.intakeIO.setBeltVoltage(this.beltVoltage.get());
      } 
    };
  }

  public Command eject() {
    var subsystem = this;
    return new Command() {
      private final LoggedTunableNumber rollerVoltage = new LoggedTunableNumber("Intake/Eject/Roller Voltage", -6);
      private final LoggedTunableNumber beltVoltage = new LoggedTunableNumber("Intake/Eject/Belt Voltage", -6);
      {
        setName("Eject");
        addRequirements(subsystem);
      }

      @Override
      public void execute() {
        subsystem.intakeIO.setRollerVoltage(this.rollerVoltage.get() * (getIntakeReversed() ? -1 : 1));
        subsystem.intakeIO.setBeltVoltage(this.beltVoltage.get());
      }

      @Override
        public InterruptionBehavior getInterruptionBehavior() {
          return InterruptionBehavior.kCancelIncoming;
        }
    };
  }

  public Command feed() {
    var subsystem = this;
    return new Command() {
      private final LoggedTunableNumber rollerVoltage = new LoggedTunableNumber("Intake/Feeding/Roller Voltage", 6);
      private final LoggedTunableNumber beltVoltage = new LoggedTunableNumber("Intake/Feeding/Belt Voltage", 6);
      {
        setName("Feed");
        addRequirements(subsystem);
      }

      public void execute() {
        subsystem.intakeIO.setRollerVoltage(this.rollerVoltage.get() * (getIntakeReversed() ? -1 : 1));
        subsystem.intakeIO.setBeltVoltage(this.beltVoltage.get());
      }
    };
  }

  public Command idle() {
    var subsystem = this;
    return new Command() {
      {
        setName("Idle");
        addRequirements(subsystem);
      }

      public void execute() {
        subsystem.intakeIO.setRollerVoltage(0);
        subsystem.intakeIO.setBeltVoltage(0);
      }
    };
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Inputs/Intake", inputs);
  }

  public boolean getIntakeReversed() {
    return intakeReversed;
  }
}
