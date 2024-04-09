// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers.kicker;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.util.LoggedTunableNumber;

public class Kicker {
  private final KickerIO kickerIO;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public static enum Goal {
    IDLE(
      () -> 0
    ),
    ANTI_DEADZONE(
      new LoggedTunableNumber("Kicker/Voltage/Anti Deadzone", 1.5)
    ),
    FEED(
      new LoggedTunableNumber("Kicker/Voltage/Feed", 1.5)
    ),
    KICK(
      new LoggedTunableNumber("Kicker/Voltage/Kick", 5)
    ),
    EJECT(
      new LoggedTunableNumber("Kicker/Voltage/Eject", -5)
    ),
    ;
    private final DoubleSupplier voltage;
    Goal(DoubleSupplier voltage) {
      this.voltage = voltage;
    }
    public double getVoltage() {
      return voltage.getAsDouble();
    }
    public void runGoal(KickerIO kickerIO) {
      kickerIO.setKickerVoltage(getVoltage());
    }
  }

  @AutoLogOutput(key = "Kicker/Goal")
  private Goal goal = Goal.IDLE;
  public Goal getGoal() {return goal;}
  public void setGoal(Goal goal) {this.goal = goal;}

  public Kicker(KickerIO kickerIO) {
    System.out.println("[Init Kicker] Instantiating Kicker");
    this.kickerIO = kickerIO;
    System.out.println("[Init Kicker] Kicker IO: " + this.kickerIO.getClass().getSimpleName());
  }

  public void periodic() {
    kickerIO.updateInputs(inputs);
    Logger.processInputs("Kicker", inputs);
    goal.runGoal(kickerIO);
  }
}
