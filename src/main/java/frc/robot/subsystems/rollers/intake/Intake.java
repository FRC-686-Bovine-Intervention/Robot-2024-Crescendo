// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers.intake;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.LoggedTunableNumber;

public class Intake {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private static final LoggedTunableNumber toggleReverseThreshold = new LoggedTunableNumber("Intake/Toggle Reverse Threshold", 0.1);
  private final DoubleSupplier forwardSpeedSupplier;

  public Intake(IntakeIO intakeIO, Supplier<ChassisSpeeds> robotRelativeSpeeds) {
    System.out.println("[Init Intake] Instantiating Intake");
    this.intakeIO = intakeIO;
    System.out.println("[Init Intake] Intake IO: " + this.intakeIO.getClass().getSimpleName());
    forwardSpeedSupplier = () -> robotRelativeSpeeds.get().vxMetersPerSecond * (intakeReversed ? -1 : 1);
  }

  public static enum Goal {
    IDLE(
      () -> 0,
      () -> 0
    ),
    ANTI_DEADZONE(
      new LoggedTunableNumber("Intake/Anti Deadzone/Roller Voltage", 1),
      new LoggedTunableNumber("Intake/Anti Deadzone/Belt Voltage", 1)
    ),
    INTAKE(
      new LoggedTunableNumber("Intake/Intaking/Roller Voltage", 6),
      new LoggedTunableNumber("Intake/Intaking/Belt Voltage", 6)
    ){
      @Override
      public void runGoal(Intake intake) {
        if(intake.forwardSpeedSupplier.getAsDouble() >= Intake.toggleReverseThreshold.get()) {
          intake.intakeReversed = !intake.intakeReversed;
        }
        super.runGoal(intake);
      }
    },
    EJECT(
      new LoggedTunableNumber("Intake/Eject/Roller Voltage", -6),
      new LoggedTunableNumber("Intake/Eject/Belt Voltage", -6)
    ),
    FEED(
      new LoggedTunableNumber("Intake/Feeding/Roller Voltage", 6),
      new LoggedTunableNumber("Intake/Feeding/Belt Voltage", 6)
    ),
    ;
    private final DoubleSupplier rollerVoltage;
    private final DoubleSupplier beltVoltage;
    Goal(DoubleSupplier rollerVoltage, DoubleSupplier beltVoltage) {
      this.rollerVoltage = rollerVoltage;
      this.beltVoltage = beltVoltage;
    }
    public double getRollerVoltage() {
      return rollerVoltage.getAsDouble();
    }
    public double getBeltVoltage() {
      return beltVoltage.getAsDouble();
    }
    public void runGoal(Intake intake) {
      intake.intakeIO.setRollerVoltage(getRollerVoltage() * (intake.intakeReversed ? -1 : 1));
      intake.intakeIO.setBeltVoltage(getBeltVoltage());
    }
  }

  @AutoLogOutput(key = "Intake/Goal")
  private Goal goal = Goal.IDLE;
  public Goal getGoal() {return goal;}
  public void setGoal(Goal goal) {this.goal = goal;}

  private boolean intakeReversed;

  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    goal.runGoal(this);
  }

  public boolean getIntakeReversed() {
    return intakeReversed;
  }
}
