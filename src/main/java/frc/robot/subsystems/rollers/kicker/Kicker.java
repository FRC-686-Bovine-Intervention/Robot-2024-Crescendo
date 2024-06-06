// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import frc.robot.util.LoggedTunableNumber;

public class Kicker extends SubsystemBase {
  private final KickerIO kickerIO;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public final InternalButton isKicking = new InternalButton();

  public Command antiDeadZone() {
    var subsystem = this;
    return new Command() {
      private final LoggedTunableNumber voltage = new LoggedTunableNumber("Kicker/Voltage/Anti Deadzone", 1.5);
      {
        setName("AntiDeadzone");
        addRequirements(subsystem);
      }

      @Override
      public void execute() {
        kickerIO.setKickerVoltage(voltage.get());
      }
    };
  }

  public Command feed() {
    var subsystem = this;
    return new Command() {
      private final LoggedTunableNumber voltage = new LoggedTunableNumber("Kicker/Voltage/Feed", 1.5);
      {
        setName("Feed");
        addRequirements(subsystem);
      }

      @Override
      public void execute() {
        kickerIO.setKickerVoltage(voltage.get());
      }
    };
  }

  public Command kick() {
    var subsystem = this;
    return new Command() {
      private final LoggedTunableNumber voltage = new LoggedTunableNumber("Kicker/Voltage/Kick", 5);
      {
        setName("Kick");
        addRequirements(subsystem);
      }

      @Override
      public void initialize() {
        isKicking.setPressed(true);
      }

      @Override
      public void execute() {
        kickerIO.setKickerVoltage(voltage.get());
      }

      @Override
      public void end(boolean interrupted) {
          isKicking.setPressed(false);
      }
    };
  }

  public Command eject() {
    var subsystem = this;
    return new Command() {
      private final LoggedTunableNumber voltage = new LoggedTunableNumber("Kicker/Voltage/Eject", -5);
      {
        setName("Eject");
        addRequirements(subsystem);
      }

      @Override
      public void execute() {
        kickerIO.setKickerVoltage(voltage.get());
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

      @Override
      public void execute() {
        kickerIO.setKickerVoltage(0);
      }
    };
  }

  public Kicker(KickerIO kickerIO) {
    System.out.println("[Init Kicker] Instantiating Kicker");
    this.kickerIO = kickerIO;
    System.out.println("[Init Kicker] Kicker IO: " + this.kickerIO.getClass().getSimpleName());
  }

  @Override
  public void periodic() {
    kickerIO.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
  }
}
