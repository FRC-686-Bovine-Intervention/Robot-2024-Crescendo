// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

public class Pivot extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public static final double POS_ZERO = 0;
  public static final double POS_AMP = 1.4864273834611863;

  private final LoggedTunableNumber pidkP = new LoggedTunableNumber("Pivot/PID/kP", 5);
  private final LoggedTunableNumber pidkI = new LoggedTunableNumber("Pivot/PID/kI", 10); 
  private final LoggedTunableNumber pidkD = new LoggedTunableNumber("Pivot/PID/kD", 0);
  private final LoggedTunableNumber pidkV = new LoggedTunableNumber("Pivot/PID/kV", 2);
  private final LoggedTunableNumber pidkA = new LoggedTunableNumber("Pivot/PID/kA", 4);
  private final LoggedTunableNumber toleranceDeg = new LoggedTunableNumber("Pivot/PID/Position Tolerance Deg", 2);
  private final ProfiledPIDController pivotPID = 
    new ProfiledPIDController(
      pidkP.get(),
      pidkI.get(),
      pidkD.get(),
      new Constraints(
          pidkV.get(),
          pidkA.get()
      )
    );

  private final LoggedTunableNumber ffkS = new LoggedTunableNumber("Pivot/FF/kS", 0);
  private final LoggedTunableNumber ffkG = new LoggedTunableNumber("Pivot/FF/kG", 0);
  private final LoggedTunableNumber ffkV = new LoggedTunableNumber("Pivot/FF/kV", 3);
  private final LoggedTunableNumber ffkA = new LoggedTunableNumber("Pivot/FF/kA", 0);
  private ArmFeedforward feedforward = 
    new ArmFeedforward(
      ffkS.get(),
      ffkG.get(),
      ffkV.get(),
      ffkA.get()
    );

  private static final Translation3d robotToPivotTranslation = 
    new Translation3d(
      Inches.of(13),
      Inches.of(0),
      Inches.of(22.665031)
    );

  private void updateTunables() {
    if(pidkP.hasChanged(hashCode()) | pidkI.hasChanged(hashCode()) | pidkD.hasChanged(hashCode())) {
        pivotPID.setPID(pidkP.get(), pidkI.get(), pidkD.get());
    }
    if(pidkV.hasChanged(hashCode()) | pidkA.hasChanged(hashCode())) {
        pivotPID.setConstraints(new Constraints(pidkV.get(), pidkA.get()));
    }
    if(toleranceDeg.hasChanged(hashCode())) {
        pivotPID.setTolerance(Units.degreesToRadians(toleranceDeg.get()));
    }
    if(ffkS.hasChanged(hashCode()) | ffkG.hasChanged(hashCode()) | ffkV.hasChanged(hashCode()) | ffkA.hasChanged(hashCode())) {
      feedforward = new ArmFeedforward(ffkS.get(), ffkG.get(), ffkV.get(), ffkA.get());
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
    Logger.recordOutput("Mechanism3d/Shooter", getRobotToPivot());
  }

  public Transform3d getRobotToPivot() {
    return new Transform3d(
      robotToPivotTranslation,
      new Rotation3d(
        0,
        inputs.pivotEncoder.positionRad,
        0
      )
    );
  }

  private final LoggedTunableNumber manualPivotVolts = new LoggedTunableNumber("Pivot/Manual Arm Volts", 2);
  public Command movePivotManually(double dir) {
    return new StartEndCommand(
        () -> pivotIO.setPivotVoltage(manualPivotVolts.get() * dir),
        () -> pivotIO.setPivotVoltage(0),
        this
    ).withName("Manual");
  }

  private Command go(DoubleSupplier pos) {
    return new ProfiledPIDCommand(
      pivotPID,
      () -> inputs.pivotEncoder.positionRad,
      pos,
      (output, setpoint) -> {
        Logger.recordOutput("Pivot/PID out", output);
        Logger.recordOutput("Pivot/Profile Position", setpoint.position);
        Logger.recordOutput("Pivot/Profile Velocity", setpoint.velocity);
        var ff = feedforward.calculate(inputs.pivotEncoder.positionRad, setpoint.velocity);
        Logger.recordOutput("Pivot/FF out", ff);
        pivotIO.setPivotVoltage(output + ff);
      },
      this
    );
  }

  public Command gotoAmp() {
    return go(() -> POS_AMP).withName("Go to Amp");
  }

  public Command gotoZero() {
    return go(() -> POS_ZERO).withName("Go to Zero");
  }

  private static final LoggedTunableNumber variableRate = new LoggedTunableNumber("Pivot/variableRate", 5);
  private double variable = 0;
  public Command gotoVariable(BooleanSupplier decrease, BooleanSupplier increase) {
    return go(() -> {
      if(decrease.getAsBoolean()) {
        variable -= Units.degreesToRadians(variableRate.get()) * Constants.dtSeconds;
      }
      if(increase.getAsBoolean()) {
        variable += Units.degreesToRadians(variableRate.get()) * Constants.dtSeconds;
      }
      return variable;
    }).withName("Go to Tunable");
  }

  public Command autoAim(Supplier<Translation2d> shootAtPos) {
    return go(() -> {
      int lowerBound = 0;
      int upperBound = 0;
      double distanceToSpeaker = shootAtPos.get().getDistance(RobotState.getInstance().getPose().getTranslation());
      for(int i = 0; i < ShooterConstants.distance.length; i++) {
        upperBound = i;
        if(distanceToSpeaker < ShooterConstants.distance[i]) {
          break;
        }
        lowerBound = i;
      }
      double t = MathUtil.inverseInterpolate(ShooterConstants.distance[lowerBound], ShooterConstants.distance[upperBound], distanceToSpeaker);
      double angle = MathUtil.interpolate(ShooterConstants.angle[lowerBound], ShooterConstants.angle[upperBound], t);
      return angle;
    }).withName("Auto Aim");
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
