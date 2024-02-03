// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.LoggedTunableNumber;

public class Pivot extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public static final double POS_ZERO = 0;
  public static final double POS_AMP = 10;

  private final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/PID/kP", 0);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/PID/kI", 0); 
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/PID/kD", 0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/PID/kV", 0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/PID/kA", 0);
  private final LoggedTunableNumber toleranceDeg = new LoggedTunableNumber("Pivot/PID/Position Tolerance Deg", 0);
  
  private final ProfiledPIDController pivotPID =  new ProfiledPIDController(
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
        pivotPID.setPID(kP.get(), kI.get(), kD.get());
    }
    if(kV.hasChanged(hashCode()) || kA.hasChanged(hashCode())) {
        pivotPID.setConstraints(new Constraints(kV.get(), kA.get()));
    }
    if(toleranceDeg.hasChanged(hashCode())) {
        pivotPID.setTolerance(Units.degreesToRadians(toleranceDeg.get()));
    }
  }

  public Pivot(PivotIO pivotIO) {
    this.pivotIO = pivotIO;
    SmartDashboard.putData("Subsystems/Pivot", this);
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    updateTunables();
    Logger.recordOutput("Mechanism3d/Shooter", 
      new Pose3d(
        new Translation3d(
          Inches.of(13),
          Inches.of(0),
          Inches.of(22.665031)
        ),
        new Rotation3d(
          0,
          inputs.pivotEncoder.positionRad,
          0
        )
      )
    );
  }

  private final LoggedTunableNumber manualPivotVolts = new LoggedTunableNumber("Pivot/Manual Arm Volts", 5);
  public Command movePivotManually(double dir) {
    return new StartEndCommand(
        () -> pivotIO.setPivotVoltage(manualPivotVolts.get() * dir),
        () -> pivotIO.setPivotVoltage(0),
        this
    ).withName("Manual");
  }

  public Command gotoAmp() {
    return new ProfiledPIDCommand(
      pivotPID,
      () -> inputs.pivotEncoder.positionRad,
      POS_AMP,
      (output, setpoint) -> pivotIO.setPivotVoltage(output),
      this
    ).withName("Go to Amp");
  }

  public Command gotoZero() {
    return new ProfiledPIDCommand(
      pivotPID,
      () -> inputs.pivotEncoder.positionRad,
      POS_ZERO,
      (output, setpoint) -> pivotIO.setPivotVoltage(output),
      this
    ).withName("Go to Zero");
  }

  public Command waitUntilAtPos() {
      return new WaitUntilCommand(pivotPID::atGoal);
  }

  public boolean isAtAngle(double angleRad) {
    return Math.abs(inputs.pivotEncoder.positionRad - angleRad) <= Units.degreesToRadians(toleranceDeg.get());
  }

  public boolean readyToFeed() {
    return isAtAngle(POS_ZERO);
  }
}
