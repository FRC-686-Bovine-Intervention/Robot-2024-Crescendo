// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;

public class Pivot extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  public static final LoggedTunableNumber toleranceDeg = new LoggedTunableNumber("Pivot/PID/Position Tolerance Deg", 1);

  public static final double POS_ZERO = Units.degreesToRadians(9);
  public static final double POS_AMP = Units.degreesToRadians(108/* .193359375 */);

  private static final Translation3d robotToPivotTranslation = 
    new Translation3d(
      Inches.of(13),
      Inches.of(0),
      Inches.of(22.665031)
    )
  ;

  public Pivot(PivotIO pivotIO, BooleanSupplier increaseRuntimeOffset, BooleanSupplier decreaseRuntimeOffset) {
    System.out.println("[Init Pivot] Instantiating Pivot");
    this.pivotIO = pivotIO;
    System.out.println("[Init Pivot] Pivot IO: " + this.pivotIO.getClass().getSimpleName());
    this.increaseRuntimeOffset = increaseRuntimeOffset;
    this.decreaseRuntimeOffset = decreaseRuntimeOffset;
    SmartDashboard.putData("Subsystems/Pivot", this);
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    Logger.recordOutput("Mechanism3d/Shooter", getRobotToPivot());
    if(increaseRuntimeOffset.getAsBoolean() && !prevInc) {
      runtimeOffset += 0.5;
    }
    if(decreaseRuntimeOffset.getAsBoolean() && !prevDec) {
      runtimeOffset -= 0.5;
    }
    prevInc = increaseRuntimeOffset.getAsBoolean();
    prevDec = decreaseRuntimeOffset.getAsBoolean();
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

  @AutoLogOutput(key = "Pivot/Runtime Offset")
  private double runtimeOffset = 0;

  private boolean prevInc;
  private final BooleanSupplier increaseRuntimeOffset;
  private boolean prevDec;
  private final BooleanSupplier decreaseRuntimeOffset;

  private boolean outtakeCommand;

  private Command go(DoubleSupplier pos) {
    var subsystem = this;
    return new Command() {
      {
        addRequirements(subsystem);
      }
      @Override
      public void initialize() {
        execute();
      }
      @Override
      public void execute() {
        pivotIO.setPivotPos(pos.getAsDouble());
      }
      @Override
      public void end(boolean interrupted) {
        pivotIO.stop();
      }
    };
  }

  private Command aim(DoubleSupplier pos) {
    var subsystem = this;
    return new Command() {
      {
        addRequirements(subsystem);
      }
      @Override
      public void initialize() {
        execute();
      }
      @Override
      public void execute() {
        outtakeCommand = true;
        pivotIO.setPivotPos(pos.getAsDouble());
      }
      @Override
      public void end(boolean interrupted) {
        outtakeCommand = false;
        pivotIO.stop();
      }
    };
  }

  public Command gotoAmp() {
    return aim(() -> POS_AMP).withName("Go to Amp");
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

  public Command autoAim(Supplier<Translation2d> FORR) {
    return aim(() -> ShooterConstants.distLerp(FORR.get().getNorm(), ShooterConstants.angle)).withName("Auto Aim");
  }

  public boolean readyToShoot() {
    return atPos() && outtakeCommand;
  }

  public boolean atPos() {
    return inputs.atGoal;
  }

  public boolean isAtAngle(double angleRad) {
    return MathUtil.isNear(angleRad, inputs.pivotEncoder.positionRad, Units.degreesToRadians(toleranceDeg.get()));
  }

  public void setCoast(boolean coast) {
    pivotIO.setCoast(coast);
  }
}
